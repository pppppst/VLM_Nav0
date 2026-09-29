from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_bridge_defaults_are_dry_run_disarmed_and_conservative():
    config = yaml.safe_load(
        (ROOT / "config/go2_bridge.yaml").read_text(encoding="utf-8")
    )
    params = config["twist_to_go2_sport_bridge"]["ros__parameters"]

    assert params["dry_run"] is True
    assert params["default_armed"] is False
    assert params["max_vx"] == 0.40
    assert params["max_vyaw"] == 0.50
    assert params["cmd_timeout"] == 0.5
    assert params["control_rate"] == 20.0
    assert params["vy_epsilon"] == 0.0001
    assert params["endpoint_loss_debounce"] > 0.0
    assert params["cmd_vel_topic"] == "/cmd_vel_bridge"
    assert params["sport_request_topic"] == "/api/sport/request"
    assert params["sport_response_topic"] == "/api/sport/response"
    assert params["require_system_ready"] is True
    assert params["system_ready_topic"] == "/vlm_nav/system_ready"
    assert params["foreign_sport_quiet_period_s"] == 2.0
    assert params["fault_on_foreign_sport_request"] is True


def test_bridge_source_contains_required_state_machine_and_unitree_api_ids():
    header = (ROOT / "include/vlm_nav/go2_bridge_state.hpp").read_text()
    source = (ROOT / "src/twist_to_go2_sport_bridge.cpp").read_text()

    for state in ("DISARMED", "ARMED", "FAULT"):
        assert state in header
    assert "last_valid_cmd_age" in header
    assert "endpoint_loss_debounce" in header
    assert "vy_epsilon" in header
    assert "1008" in source
    assert "1003" in source
    assert "header.identity.api_id" in source
    assert '\\"y\\":0.0' in source
    assert "dry_run" in source
    assert "reset_fault" in source
    assert "system_ready_" in source
    assert "SYSTEM_READY gate is false" in source
    assert "go2_sport_interface_ready" in source
    assert "Sport interface endpoint absent beyond debounce" in source
    assert "control_armed_pub_" in source
    assert "publisher_gid" in source
    assert "foreign Sport request observed while ARMED" in source
    destructor = source.split("~TwistToGo2SportBridge", 1)[1].split("private:", 1)[0]
    assert "BridgeState::ARMED" in destructor
    assert "publish_api_request(kStopMoveApiId" not in destructor


def test_bridge_fault_contract_is_covered_by_native_tests():
    test_source = (ROOT / "test/test_go2_bridge_state.cpp").read_text()

    for behavior in (
        "StartsDisarmed",
        "RejectsNonFiniteCommand",
        "RejectsLateralVelocity",
        "ClampsAcceptedCommand",
        "CommandTimeoutFaultsAndStopsOnce",
        "EndpointLossUsesDebounce",
        "FaultNeverAutomaticallyRearms",
        "DisarmStopsOnce",
        "ExternalSafetyGateFaultsAndStopsOnce",
        "RejectsUnsupportedTwistAxes",
        "DisarmedRejectsAndDoesNotCacheNonzeroCommand",
        "ArmDoesNotRequireOrActivateCachedCommand",
    ):
        assert behavior in test_source
