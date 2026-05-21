"""Test whether the deployed GR00T server supports RTC options.

Sends two identical observations — one with options=None and one with RTC options —
and compares the responses. If the server forwards options to the model, the
actions should differ (RTC uses the previous action as inpainting constraint).

Run:
    PYTHONPATH=. uv run --with pyzmq --with msgpack \
        python policy/examples/test_rtc_server_support.py
"""

from __future__ import annotations

import sys
import time

import numpy as np

# --- Config ---
GROOT_HOST = "172.31.11.129"
GROOT_PORT = 30555
TIMEOUT_MS = 15000


def main():
    from policy.gr00t.transport import Gr00tZmqTransport

    transport = Gr00tZmqTransport(host=GROOT_HOST, port=GROOT_PORT, timeout_ms=TIMEOUT_MS)
    transport.connect()

    # Step 1: Ping
    print(f"Connecting to {GROOT_HOST}:{GROOT_PORT}...")
    try:
        ping = transport.call("ping")
        print(f"  ✓ Ping: {ping}")
    except Exception as e:
        print(f"  ✗ Ping failed: {e}")
        sys.exit(1)

    # Step 2: Get modality config to build a valid observation
    print("\nFetching modality config...")
    config = transport.call("get_modality_config")
    print(f"  Keys: {list(config.keys())}")

    # Extract expected shapes
    state_config = config.get("state", {})
    video_config = config.get("video", {})
    action_config = config.get("action", {})
    lang_config = config.get("language", {})

    state_keys = _get_modality_keys(state_config)
    video_keys = _get_modality_keys(video_config)
    action_keys = _get_modality_keys(action_config)
    lang_keys = _get_modality_keys(lang_config)
    action_horizon = len(_get_delta_indices(action_config))

    print(f"  State keys: {state_keys}")
    print(f"  Video keys: {video_keys}")
    print(f"  Action keys: {action_keys}")
    print(f"  Language keys: {lang_keys}")
    print(f"  Action horizon: {action_horizon}")

    # Step 3: Build a dummy observation
    obs = _build_dummy_obs(config, state_keys, video_keys, lang_keys)
    print(f"\nBuilt dummy observation with keys: {_obs_summary(obs)}")

    # Step 4: Call get_action WITHOUT options (baseline)
    print("\n--- Test 1: get_action with options=None ---")
    t0 = time.monotonic()
    try:
        resp_none = transport.call("get_action", {"observation": obs, "options": None})
        dt = (time.monotonic() - t0) * 1000
        print(f"  ✓ Response in {dt:.0f}ms")
        action_none = _extract_action(resp_none)
        _print_action_summary(action_none)
    except Exception as e:
        print(f"  ✗ Failed: {e}")
        sys.exit(1)

    # Step 5: Call get_action WITH RTC options (but no previous action)
    # If the server passes options through, it might error because
    # RTC requires "action" in action_input. Or it might just ignore options.
    rtc_options = {
        "action_horizon": action_horizon,
        "rtc_overlap_steps": min(16, action_horizon // 2),
        "rtc_frozen_steps": 2,
        "rtc_ramp_rate": 3.0,
    }
    print(f"\n--- Test 2: get_action with RTC options (no prev action) ---")
    print(f"  Options: {rtc_options}")
    t0 = time.monotonic()
    try:
        resp_opts = transport.call("get_action", {"observation": obs, "options": rtc_options})
        dt = (time.monotonic() - t0) * 1000
        print(f"  ✓ Response in {dt:.0f}ms (server accepted options without error)")
        action_opts = _extract_action(resp_opts)
        _print_action_summary(action_opts)
    except RuntimeError as e:
        dt = (time.monotonic() - t0) * 1000
        print(f"  ✗ Server error in {dt:.0f}ms: {e}")
        if "rtc" in str(e).lower() or "action" in str(e).lower():
            print("  → Server IS reading options (tried to use RTC but failed due to missing prev action)")
            print("  → RTC IS supported on the server side!")
        else:
            print("  → Unclear — error might be unrelated to RTC")
        transport.close()
        transport = Gr00tZmqTransport(host=GROOT_HOST, port=GROOT_PORT, timeout_ms=TIMEOUT_MS)
        transport.connect()
        action_opts = None

    # Step 6: If both succeeded, compare actions
    if action_none is not None and action_opts is not None:
        print("\n--- Comparison ---")
        # With the same observation and no previous action, if options are ignored,
        # both responses will be driven by the same random seed → likely similar but
        # not identical (stochastic model). We can't definitively tell from this alone.
        # But we can check: did the server at least ACCEPT options without error?
        print("  Server accepted options without error.")
        print("  (Cannot definitively confirm RTC is active without prev action in obs)")

    # Step 7: Full RTC test — pass previous action IN the observation
    print(f"\n--- Test 3: get_action with RTC options + previous action in observation ---")
    # The RTC mechanism checks if "action" is in action_input.
    # PR #320 merges previous action into the observation dict.
    obs_with_action = dict(obs)
    if action_none is not None:
        # Merge action arrays into obs (this is how PR #320's RTCPolicyWrapper does it)
        for key, arr in action_none.items():
            obs_with_action[key] = arr

    t0 = time.monotonic()
    try:
        resp_rtc = transport.call(
            "get_action", {"observation": obs_with_action, "options": rtc_options}
        )
        dt = (time.monotonic() - t0) * 1000
        print(f"  ✓ Response in {dt:.0f}ms")
        action_rtc = _extract_action(resp_rtc)
        _print_action_summary(action_rtc)

        # Compare: if RTC is active, the result should differ from baseline
        if action_none is not None:
            _compare_actions(action_none, action_rtc)

    except RuntimeError as e:
        dt = (time.monotonic() - t0) * 1000
        print(f"  ✗ Server error in {dt:.0f}ms: {e}")
        if "rtc" in str(e).lower() or "overlap" in str(e).lower():
            print("  → Server tried RTC but something went wrong")
            print("  → RTC plumbing EXISTS on server, but may need adjustment")
        else:
            print(f"  → Server rejected observation with action keys: {e}")

    # Step 8: Test denoising_steps option (simpler — just changes num inference steps)
    print(f"\n--- Test 4: get_action with denoising_steps option ---")
    denoise_options = {"denoising_steps": 8}
    t0 = time.monotonic()
    try:
        resp_denoise = transport.call(
            "get_action", {"observation": obs, "options": denoise_options}
        )
        dt = (time.monotonic() - t0) * 1000
        print(f"  ✓ Response in {dt:.0f}ms")
        print("  Server accepted denoising_steps option")
    except RuntimeError as e:
        dt = (time.monotonic() - t0) * 1000
        print(f"  Response in {dt:.0f}ms: {e}")

    transport.close()
    print("\n✓ Done.")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _get_modality_keys(section: dict) -> list[str]:
    if not isinstance(section, dict):
        return []
    # Handle ModalityConfig objects (deserialized from msgpack)
    if hasattr(section, "modality_keys"):
        return list(section.modality_keys)
    as_json = section.get("as_json", section)
    if isinstance(as_json, dict):
        keys = as_json.get("modality_keys", [])
        return list(keys) if isinstance(keys, list) else []
    return []


def _get_delta_indices(section: dict) -> list[int]:
    if not isinstance(section, dict):
        return []
    if hasattr(section, "delta_indices"):
        return list(section.delta_indices)
    as_json = section.get("as_json", section)
    if isinstance(as_json, dict):
        return list(as_json.get("delta_indices", []))
    return []


def _build_dummy_obs(
    config: dict, state_keys: list[str], video_keys: list[str], lang_keys: list[str]
) -> dict:
    """Build a minimal valid observation for the server."""
    obs: dict = {}

    # State: (B=1, T=1, D) — use 6 DOF as default, 12 for dual-arm
    state_dict = {}
    for key in state_keys:
        # Guess dimension from key name
        if "joint" in key.lower():
            dim = 6
        elif "gripper" in key.lower():
            dim = 1
        elif "eef" in key.lower() or "tcp" in key.lower():
            dim = 9  # xyz + rot6d
        else:
            dim = 6
        state_dict[key] = np.zeros((1, 1, dim), dtype=np.float32)
    obs["state"] = state_dict

    # Video: (B=1, T=1, H, W, C)
    video_dict = {}
    for key in video_keys:
        video_dict[key] = np.zeros((1, 1, 224, 224, 3), dtype=np.uint8)
    obs["video"] = video_dict

    # Language
    lang_dict = {}
    for key in lang_keys:
        lang_dict[key] = [["pick up the object"]]
    obs["language"] = lang_dict

    return obs


def _obs_summary(obs: dict) -> str:
    parts = []
    for modality, content in obs.items():
        if isinstance(content, dict):
            for k, v in content.items():
                if isinstance(v, np.ndarray):
                    parts.append(f"{modality}.{k}: {v.shape} {v.dtype}")
                else:
                    parts.append(f"{modality}.{k}: {type(v).__name__}")
        else:
            parts.append(f"{modality}: {type(content).__name__}")
    return "\n    ".join(parts)


def _extract_action(response) -> dict[str, np.ndarray] | None:
    """Extract action dict from GR00T response (action, info) tuple."""
    if isinstance(response, (list, tuple)) and len(response) == 2:
        action = response[0]
        if isinstance(action, dict):
            return {k: v for k, v in action.items() if isinstance(v, np.ndarray)}
    return None


def _print_action_summary(action: dict[str, np.ndarray] | None):
    if action is None:
        print("  No action extracted")
        return
    for key, arr in action.items():
        print(f"  {key}: shape={arr.shape} range=[{arr.min():.4f}, {arr.max():.4f}]")


def _compare_actions(a1: dict[str, np.ndarray], a2: dict[str, np.ndarray]):
    """Compare two action dicts and report similarity."""
    for key in a1:
        if key not in a2:
            continue
        arr1, arr2 = a1[key], a2[key]
        if arr1.shape != arr2.shape:
            print(f"  {key}: shape mismatch {arr1.shape} vs {arr2.shape}")
            continue
        diff = np.abs(arr1 - arr2)
        max_diff = diff.max()
        mean_diff = diff.mean()
        if max_diff < 1e-6:
            print(f"  {key}: IDENTICAL (max_diff={max_diff:.2e}) — RTC likely NOT active")
        else:
            print(
                f"  {key}: DIFFERENT (max_diff={max_diff:.4f}, mean_diff={mean_diff:.4f}) "
                f"— RTC may be active (or stochastic noise)"
            )


if __name__ == "__main__":
    main()
