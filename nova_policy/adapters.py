from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from .models import ACTPolicy, JsonValue, PolicySpec


class PolicyAdapter(Protocol):
    """Maps typed SDK policy objects to the policy-service contract."""

    def service_policy_payload(self, *, device: str | None = None) -> dict[str, JsonValue]: ...


@dataclass(slots=True, frozen=True)
class ACTAdapter:
    """ACT v1 adapter for the phase-2/3 policy service contract."""

    policy: ACTPolicy

    def service_policy_payload(self, *, device: str | None = None) -> dict[str, JsonValue]:
        return {
            "kind": self.policy.kind,
            "path": self.policy.path,
            "n_action_steps": self.policy.n_action_steps,
            "device": device,
        }


def adapter_for_policy(policy: PolicySpec) -> PolicyAdapter:
    if isinstance(policy, ACTPolicy):
        return ACTAdapter(policy)
    raise TypeError(f"Unsupported policy type: {type(policy)!r}")
