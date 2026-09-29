from pathlib import Path

import yaml


def test_rviz_uses_native_grayscale_costmaps():
    config = yaml.safe_load(
        (Path(__file__).parents[1] / "config/go2_costmaps.rviz").read_text()
    )
    displays = config["Visualization Manager"]["Displays"]
    by_name = {item["Name"]: item for item in displays}

    assert by_name["Global Costmap"]["Class"] == "rviz_default_plugins/Map"
    assert by_name["Global Costmap"]["Topic"]["Value"] == "/global_costmap/costmap"
    assert by_name["Local Costmap"]["Class"] == "rviz_default_plugins/Map"
    assert by_name["Local Costmap"]["Topic"]["Value"] == "/local_costmap/costmap"
    assert by_name["Global Costmap"]["Color Scheme"] == "map"
    assert by_name["Local Costmap"]["Color Scheme"] == "map"
    assert by_name["Robot footprint"]["Topic"]["Value"] == "/local_costmap/published_footprint"
    assert by_name["Robot +X direction"]["Shape"]["Shaft Length"] == 0.20
    assert by_name["Global Plan"]["Topic"]["Value"] == "/plan"
    assert by_name["Local Plan"]["Topic"]["Value"] == "/local_plan"
    assert by_name["Camera RGB (1 Hz preview)"]["Topic"]["Value"] == "/vlm_nav/camera_preview"
    assert by_name["Camera RGB (1 Hz preview)"]["Topic"]["Reliability Policy"] == "Best Effort"

    tools = config["Visualization Manager"]["Tools"]
    by_class = {tool["Class"]: tool for tool in tools}
    assert by_class["rviz_default_plugins/PublishPoint"]["Topic"]["Value"] == "/clicked_point"
    assert by_class["rviz_default_plugins/SetGoal"]["Topic"]["Value"] == "/goal_pose"
