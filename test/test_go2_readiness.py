from vlm_nav.go2_readiness import ReadinessInputs, evaluate_readiness


def healthy_inputs(**overrides):
    values = dict(
        clock=True,
        dds=True,
        lidar=True,
        imu=True,
        fast_lio=True,
        base_tf=True,
        sport_bridge=True,
        obstacle_chain=True,
        slam=True,
        map_to_odom=True,
        nav2=True,
        rgb=True,
        raw_depth=True,
        color_camera_info=True,
        depth_camera_info=True,
        rgbd_sync=True,
        camera_geometry=True,
        units_verified=True,
        camera_calibrated=True,
        vlm_api=True,
    )
    values.update(overrides)
    return ReadinessInputs(**values)


def test_readiness_states_are_independent_and_layered():
    result = evaluate_readiness(healthy_inputs())
    assert result.system_ready is True
    assert result.nav_ready is True
    assert result.vlm_input_ready is True
    assert result.vlm_autonomy_ready is True
    assert result.vlm_gate_ready is True

    no_api = evaluate_readiness(healthy_inputs(vlm_api=False))
    assert no_api.system_ready is True
    assert no_api.nav_ready is True
    assert no_api.vlm_input_ready is True
    assert no_api.vlm_autonomy_ready is False
    assert no_api.vlm_gate_ready is False

    no_units = evaluate_readiness(healthy_inputs(units_verified=False))
    assert no_units.vlm_input_ready is True
    assert no_units.vlm_autonomy_ready is False

    no_nav = evaluate_readiness(healthy_inputs(nav2=False))
    assert no_nav.system_ready is True
    assert no_nav.nav_ready is False
    assert no_nav.vlm_gate_ready is False


def test_motion_and_vlm_require_explicit_independent_enable_flags():
    result = evaluate_readiness(
        healthy_inputs(), control_armed=False, vlm_enabled=False
    )
    assert result.nav_motion_allowed is False
    assert result.vlm_navigation_allowed is False

    armed = evaluate_readiness(
        healthy_inputs(), control_armed=True, vlm_enabled=False
    )
    assert armed.nav_motion_allowed is True
    assert armed.vlm_navigation_allowed is False

    full = evaluate_readiness(
        healthy_inputs(), control_armed=True, vlm_enabled=True
    )
    assert full.vlm_navigation_allowed is True


def test_runtime_supervisors_have_single_owned_readiness_topics():
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    nav_source = (root / "vlm_nav/go2_nav_supervisor.py").read_text()
    nav_launch = (root / "launch/go2_navigation.launch.py").read_text()
    assert '"/vlm_nav/nav_ready"' in nav_source
    assert "GetState" in nav_source
    assert "PRIMARY_STATE_ACTIVE" in nav_source
    assert "system_ready_receipt" in nav_source
    assert 'count_publishers("/vlm_nav/system_ready") == 1' in nav_source
    assert "lifecycle_receipts" in nav_source
    assert "go2_nav_supervisor" in nav_launch


def test_nav_supervisor_does_not_shadow_rclpy_clients_property():
    from pathlib import Path

    source = (Path(__file__).resolve().parents[1] / "vlm_nav/go2_nav_supervisor.py").read_text()
    assert "self.lifecycle_clients =" in source
    assert "self.clients =" not in source
