# Go2 integration evidence (2026-10-07)

REPO=/home/isee-pst/unitree_ros2/VLM_Nav
ROS_SRC=/home/isee-pst/unitree_ros2/VLM_Nav/cooperative_ws/src
Original branch: VLM_nav_go. Implementation branch: go2-cooperative-task-layer.
No applicable AGENTS.md found. Input plans were the only untracked files.

| Existing entry/interface | Cooperative use | Failure / field verification |
|---|---|---|
| scripts/manualnav2.sh, scripts/VLMNav-go.sh | Preserve original single robot scripts; cooperative entry must not start their autonomous VLM loop | These scripts probe hardware / ARM; not executed here |
| config/robot_go2.yaml | Raw D435 `/camera/camera/color/image_raw`, `/camera/camera/depth/image_rect_raw` and both camera_info topics | RGB-D synchronization, depth-unit report, calibrated camera TF required |
| vlm_nav.geometry.RealSenseGeometry / depth_at_pixel_with_reason / project_pixel | Image-time depth + TF map projection | Invalid depth/TF yields detected-without-position |
| vlm_nav.vlm_client, LatestFrameWorker | Validated API results and bounded asynchronous inference | Existing request is single-target; add ABC envelope without planner calls |
| NavigateToPose /navigate_to_pose, ComputePathToPose /compute_path_to_pose, Spin /spin | New adapter owns action handles | Bind task/command/goal; cancel ACK alone is insufficient |
| map→base_link, /odometry | Map pose and independent stationary confirmation | Never label odometry as map pose |
| /local_costmap/costmap_raw, /map | Local approach candidates | Validate full footprint and local path |
| Executing Nav2 path | RegionGuard input | No existing cooperative path subscription found; live requires verified fresh path source |
| /vlm_nav/nav_ready, /vlm_nav/control_armed | Preserve external gates | No auto ARM; live field validation required |
| /vlm_nav/obstacle_cloud | Existing marking + clearing, measured footprint ±0.38×±0.18 | Filter height -0.20..0.20: peer-body detection and rear coverage need field checks; unchanged |
| multi_agent_comm/go2_agent_node | Keep /multi_agent/agents/go2/state and old multi_agent_msgs | Publishes odometry, no cooperative READY semantics |
| config/multi_agent_bridge.yaml | Keep old route file; separate pending task routes | domain_bridge 0.5.0, reversed supported by installed parser header |
| config/cyclonedds_go2_multi_agent_bridge.xml | Reuse domain-specific NIC restriction | Domain0=enp4s0, Domain40=wlo1; discovery isolation remains field acceptance |

Python: /home/isee-pst/venv/co-nav-real/bin/python, Python 3.10.12, system-site-packages.
Source Humble, then legacy workspace, then **go2_ws last**, then cooperative install.
The legacy workspace shadows the newer Go2 Sport binary if sourced last: its binary
lacks sport_response_topic, causing baseline runtime test failure. Keep ROS PYTHONPATH
when prepending the repository. No system packages or existing overlays were modified.

New package base paths must both be passed to colcon because the root is itself a ROS package.
Baseline uses Domain31 + ROS_LOCALHOST_ONLY=1; runtime Sport test is dry_run and remapped.
Production deployment remains Domain0, no 0↔31 bridge.
