from glob import glob
from setuptools import find_packages, setup


package_name = "vlm_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["pytest.ini"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        (
            "share/" + package_name + "/config",
            glob("config/*.yaml") + glob("config/*.rviz"),
        ),
        (
            "share/" + package_name + "/scripts",
            glob("scripts/*.sh") + glob("scripts/*.py"),
        ),
        (
            "share/" + package_name + "/behavior_trees",
            glob("behavior_trees/*.xml"),
        ),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="robot@example.com",
    description="VLM grounded RGB-D navigation for ROS 2 and Nav2",
    license="MIT",
    entry_points={
        "console_scripts": [
            "fastlio_odom_adapter = vlm_nav.fastlio_odom_adapter:main",
            "go2_readiness_waiter = vlm_nav.go2_readiness_waiter:main",
            "minimal_cloud_probe = vlm_nav.minimal_cloud_probe:main",
            "go2_render_fastlio_config = vlm_nav.go2_config_renderer:main",
            "go2_sensor_preflight = vlm_nav.go2_sensor_preflight:main",
            "go2_camera_preflight = vlm_nav.go2_camera_preflight:main",
            "go2_derive_candidates = vlm_nav.go2_evidence:main",
            "go2_gate5_pulse = vlm_nav.go2_gate5_pulse:main",
            "go2_safety_supervisor = vlm_nav.go2_safety_supervisor:main",
            "go2_nav_supervisor = vlm_nav.go2_nav_supervisor:main",
            "vlm_navigator = vlm_nav.vlm_navigator:main",
        ]
    },
)
