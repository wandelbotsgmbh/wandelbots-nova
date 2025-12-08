"""Utility functions for Nova viewer implementations."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Sequence

import numpy as np

if TYPE_CHECKING:
    from nova import api
    from nova.actions import Action

logger = logging.getLogger(__name__)


def downsample_trajectory(
    trajectory: api.models.JointTrajectory,
    sample_interval_ms: float = 50.0,
    curvature_weight: float = 0.7,
) -> api.models.JointTrajectory:
    """Downsample a trajectory adaptively based on time and joint movement.

    This function reduces the number of samples in a trajectory while preserving
    visual fidelity. The target sample count is determined by the trajectory duration
    and the desired sample interval. Samples are distributed adaptively, keeping more
    samples in regions with larger joint movements (high curvature/acceleration) and
    fewer samples in regions with subtle movements (straight segments).

    The algorithm:
    1. Computes target sample count from trajectory duration and sample_interval_ms
    2. Computes the "importance" of each sample based on joint position changes
    3. Samples points based on importance distribution, biasing toward high-curvature regions
    4. Always preserves the first and last samples

    Args:
        trajectory: The joint trajectory to downsample
        sample_interval_ms: Target time interval between samples in milliseconds.
            Lower values = more samples = higher fidelity but more data.
            Higher values = fewer samples = lower fidelity but faster rendering.
            The actual interval may vary due to adaptive curvature-based sampling.
            (default: 50.0ms, equivalent to 20 samples/second)
        curvature_weight: Weight for curvature-based importance (0-1).
            Higher values prioritize keeping samples at high-curvature points.
            Lower values result in more uniform time-based sampling. (default: 0.7)

    Returns:
        A new JointTrajectory with reduced samples, or the original if it's
        already at or below the target sample rate.
    """
    from nova import api

    n_samples = len(trajectory.joint_positions)

    if n_samples <= 2:
        return trajectory

    # Compute trajectory duration in milliseconds
    duration_ms = (trajectory.times[-1] - trajectory.times[0]) * 1000.0

    if duration_ms <= 0:
        return trajectory

    # Compute target number of samples based on duration and sample interval
    target_samples = max(2, int(duration_ms / sample_interval_ms) + 1)

    # If already at or below target, return as-is
    if n_samples <= target_samples:
        return trajectory

    # Convert joint positions to numpy array for efficient computation
    # Shape: (n_samples, n_joints)
    joint_array = np.array([list(jp.root) for jp in trajectory.joint_positions])

    # Compute importance scores based on joint movement
    importance = _compute_sample_importance(joint_array, curvature_weight)

    # Select indices based on importance-weighted distribution
    selected_indices = _select_samples_by_importance(importance, target_samples)

    # Build the downsampled trajectory
    downsampled_positions = [trajectory.joint_positions[i] for i in selected_indices]
    downsampled_times = [trajectory.times[i] for i in selected_indices]
    downsampled_locations = [trajectory.locations[i] for i in selected_indices]

    logger.debug(
        "Downsampled trajectory from %d to %d samples (%.1f%% reduction, %.1fms interval)",
        n_samples,
        len(selected_indices),
        (1 - len(selected_indices) / n_samples) * 100,
        duration_ms / max(1, len(selected_indices) - 1),
    )

    return api.models.JointTrajectory(
        joint_positions=downsampled_positions,
        times=downsampled_times,
        locations=downsampled_locations,
    )


def _compute_sample_importance(joint_array: np.ndarray, curvature_weight: float) -> np.ndarray:
    """Compute importance scores for each sample in the trajectory.

    Importance is based on:
    1. First derivative (velocity) - captures speed of joint changes
    2. Second derivative (acceleration/curvature) - captures direction changes

    Args:
        joint_array: Array of joint positions, shape (n_samples, n_joints)
        curvature_weight: Weight for curvature vs velocity importance

    Returns:
        Array of importance scores, shape (n_samples,)
    """
    n_samples = joint_array.shape[0]

    if n_samples < 3:
        return np.ones(n_samples)

    # Compute first derivative (velocity) - magnitude of joint changes
    velocity = np.diff(joint_array, axis=0)  # Shape: (n_samples-1, n_joints)
    velocity_magnitude = np.linalg.norm(velocity, axis=1)

    # Compute second derivative (acceleration/curvature)
    acceleration = np.diff(velocity, axis=0)  # Shape: (n_samples-2, n_joints)
    acceleration_magnitude = np.linalg.norm(acceleration, axis=1)

    # Pad to match original length
    # For velocity: assign to midpoint between samples
    velocity_importance = np.zeros(n_samples)
    velocity_importance[:-1] += velocity_magnitude
    velocity_importance[1:] += velocity_magnitude
    velocity_importance[1:-1] /= 2  # Average for interior points

    # For acceleration: assign to the sample where curvature occurs
    acceleration_importance = np.zeros(n_samples)
    acceleration_importance[1:-1] = acceleration_magnitude

    # Normalize both to [0, 1] range
    if velocity_importance.max() > 0:
        velocity_importance /= velocity_importance.max()
    if acceleration_importance.max() > 0:
        acceleration_importance /= acceleration_importance.max()

    # Combine with weighting
    importance = (
        1 - curvature_weight
    ) * velocity_importance + curvature_weight * acceleration_importance

    # Ensure minimum importance to avoid completely skipping segments
    importance = np.maximum(importance, 0.1)

    # First and last points are always maximally important
    importance[0] = 1.0
    importance[-1] = 1.0

    return importance


def _select_samples_by_importance(importance: np.ndarray, target_samples: int) -> list[int]:
    """Select sample indices based on importance distribution.

    Uses cumulative importance to ensure samples are distributed according to
    the importance weights, while guaranteeing first and last samples are included.

    Args:
        importance: Array of importance scores
        target_samples: Number of samples to select

    Returns:
        Sorted list of selected indices
    """
    n_samples = len(importance)

    if target_samples >= n_samples:
        return list(range(n_samples))

    if target_samples <= 2:
        return [0, n_samples - 1] if n_samples > 1 else [0]

    # Compute cumulative importance
    cumulative = np.cumsum(importance)
    cumulative /= cumulative[-1]  # Normalize to [0, 1]

    # Generate target positions in cumulative space
    # Exclude 0 and 1 since we'll add first and last explicitly
    target_positions = np.linspace(0, 1, target_samples)

    # Find indices that best match target positions
    selected_indices = set()
    selected_indices.add(0)  # Always include first
    selected_indices.add(n_samples - 1)  # Always include last

    for pos in target_positions[1:-1]:  # Skip first and last
        # Find the index where cumulative importance crosses this position
        idx = np.searchsorted(cumulative, pos)
        idx = min(idx, n_samples - 1)
        selected_indices.add(idx)

    # If we still need more points, fill in uniformly
    while len(selected_indices) < target_samples:
        # Find the largest gap between consecutive selected indices
        sorted_indices = sorted(selected_indices)
        max_gap = 0
        gap_start = 0
        for i in range(len(sorted_indices) - 1):
            gap = sorted_indices[i + 1] - sorted_indices[i]
            if gap > max_gap:
                max_gap = gap
                gap_start = sorted_indices[i]
        if max_gap <= 1:
            break
        # Add midpoint of largest gap
        selected_indices.add(gap_start + max_gap // 2)

    return sorted(selected_indices)


def extract_collision_setups_from_actions(
    actions: Sequence[Action],
) -> dict[str, api.models.CollisionSetup]:
    """Extract unique collision scenes from a list of actions.

    Args:
        actions: List of actions to extract collision scenes from

    Returns:
        Dictionary mapping collision scene IDs to CollisionScene objects
    """
    from nova.actions.motions import CollisionFreeMotion, Motion

    collision_scenes: dict[str, api.models.CollisionSetup] = {}

    for i, action in enumerate(actions):
        # Check if action is a motion with collision_scene attribute
        if isinstance(action, (Motion, CollisionFreeMotion)) and action.collision_setup is not None:
            # Generate a deterministic ID based on action index and type
            scene_id = f"action_{i}_{type(action).__name__}_scene"
            collision_scenes[scene_id] = action.collision_setup

    return collision_scenes
