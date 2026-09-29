"""Layered Go2 readiness gates without coupling motion and VLM enablement."""

from dataclasses import dataclass


@dataclass(frozen=True)
class ReadinessInputs:
    clock: bool
    dds: bool
    lidar: bool
    imu: bool
    fast_lio: bool
    base_tf: bool
    sport_bridge: bool
    obstacle_chain: bool
    slam: bool
    map_to_odom: bool
    nav2: bool
    rgb: bool
    raw_depth: bool
    color_camera_info: bool
    depth_camera_info: bool
    rgbd_sync: bool
    camera_geometry: bool
    units_verified: bool
    camera_calibrated: bool
    vlm_api: bool


@dataclass(frozen=True)
class ReadinessResult:
    system_ready: bool
    nav_ready: bool
    vlm_input_ready: bool
    vlm_autonomy_ready: bool
    vlm_gate_ready: bool
    control_armed: bool
    vlm_enabled: bool
    nav_motion_allowed: bool
    vlm_navigation_allowed: bool


def evaluate_readiness(
    inputs: ReadinessInputs,
    *,
    control_armed: bool = False,
    vlm_enabled: bool = False,
) -> ReadinessResult:
    system_ready = all(
        (
            inputs.clock,
            inputs.dds,
            inputs.lidar,
            inputs.imu,
            inputs.fast_lio,
            inputs.base_tf,
            inputs.sport_bridge,
        )
    )
    nav_ready = system_ready and all(
        (inputs.obstacle_chain, inputs.slam, inputs.map_to_odom, inputs.nav2)
    )
    vlm_input_ready = nav_ready and all(
        (
            inputs.rgb,
            inputs.raw_depth,
            inputs.color_camera_info,
            inputs.depth_camera_info,
            inputs.rgbd_sync,
            inputs.camera_geometry,
            inputs.camera_calibrated,
        )
    )
    vlm_autonomy_ready = (
        vlm_input_ready and inputs.units_verified and inputs.vlm_api
    )
    return ReadinessResult(
        system_ready=system_ready,
        nav_ready=nav_ready,
        vlm_input_ready=vlm_input_ready,
        vlm_autonomy_ready=vlm_autonomy_ready,
        vlm_gate_ready=vlm_autonomy_ready,
        control_armed=control_armed,
        vlm_enabled=vlm_enabled,
        nav_motion_allowed=nav_ready and control_armed,
        vlm_navigation_allowed=(
            vlm_autonomy_ready and control_armed and vlm_enabled
        ),
    )
