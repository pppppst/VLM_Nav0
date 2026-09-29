cmake_minimum_required(VERSION 3.8)
project(vlm_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_plugins REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_api QUIET)
find_package(unitree_go QUIET)
find_package(PCL REQUIRED COMPONENTS filters)
find_package(pluginlib REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_library(componentwise_min_speed_generator SHARED
  src/componentwise_min_speed_generator.cpp)
target_compile_features(componentwise_min_speed_generator PUBLIC cxx_std_17)
target_include_directories(componentwise_min_speed_generator
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(componentwise_min_speed_generator
  dwb_core
  dwb_plugins
  nav_2d_msgs
  pluginlib
)
pluginlib_export_plugin_description_file(dwb_core componentwise_min_speed_plugins.xml)

add_library(obstacle_cloud_filter_pipeline STATIC src/obstacle_cloud_filter_pipeline.cpp)
target_compile_features(obstacle_cloud_filter_pipeline PUBLIC cxx_std_17)
target_include_directories(obstacle_cloud_filter_pipeline
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(obstacle_cloud_filter_pipeline
  pcl_conversions
  sensor_msgs
)
target_link_libraries(obstacle_cloud_filter_pipeline ${PCL_LIBRARIES})

add_executable(obstacle_cloud_filter src/obstacle_cloud_filter.cpp)
target_compile_features(obstacle_cloud_filter PUBLIC cxx_std_17)
target_include_directories(obstacle_cloud_filter PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(obstacle_cloud_filter
  diagnostic_msgs
  diagnostic_updater
  message_filters
  pcl_ros
  rclcpp
  rmw
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(obstacle_cloud_filter
  obstacle_cloud_filter_pipeline
  ${PCL_LIBRARIES}
)

if(unitree_api_FOUND)
  add_executable(twist_to_go2_sport_bridge src/twist_to_go2_sport_bridge.cpp)
  target_compile_features(twist_to_go2_sport_bridge PUBLIC cxx_std_17)
  target_include_directories(twist_to_go2_sport_bridge
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(twist_to_go2_sport_bridge
    geometry_msgs
    rclcpp
    rmw
    std_msgs
    std_srvs
    unitree_api
  )
else()
  message(WARNING
    "unitree_api is unavailable: Ranger targets remain buildable, but the "
    "Go2 Sport bridge will not be built. Import the pinned unitree_ros2 dependency."
  )
endif()

if(unitree_go_FOUND)
  add_executable(lowstate_imu_adapter src/lowstate_imu_adapter.cpp)
  target_compile_features(lowstate_imu_adapter PUBLIC cxx_std_17)
  ament_target_dependencies(lowstate_imu_adapter
    rclcpp
    sensor_msgs
    unitree_go
  )
else()
  message(WARNING
    "unitree_go is unavailable: the Go2 /lowstate to /body_imu adapter will not be built."
  )
endif()

set(vlm_nav_runtime_targets obstacle_cloud_filter)
if(TARGET twist_to_go2_sport_bridge)
  list(APPEND vlm_nav_runtime_targets twist_to_go2_sport_bridge)
endif()
if(TARGET lowstate_imu_adapter)
  list(APPEND vlm_nav_runtime_targets lowstate_imu_adapter)
endif()
install(
  TARGETS ${vlm_nav_runtime_targets}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS componentwise_min_speed_generator LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(
  PROGRAMS
    scripts/fastlio_odom_adapter
    scripts/go2_camera_preflight
    scripts/go2_derive_candidates
    scripts/go2_gate5_pulse
    scripts/go2_footprint_sweep_debug.py
    scripts/go2_nav_supervisor
    scripts/go2_readiness_waiter
    scripts/go2_render_fastlio_config
    scripts/go2_safety_supervisor
    scripts/go2_sensor_preflight
    scripts/minimal_cloud_probe
    scripts/manualnav2.sh
    scripts/movetest.sh
    scripts/run_go2_preflight.sh
    scripts/inspect_go2_sport_publishers.sh
    scripts/stopnav2.sh
    scripts/stopgo.sh
    scripts/VLMNav-go.sh
    scripts/vlm_navigator
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config behavior_trees DESTINATION share/${PROJECT_NAME})
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})
install(FILES go2_dependencies.repos DESTINATION share/${PROJECT_NAME})
install(
  DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.sh" PATTERN "*.py"
)
install(FILES package.xml pytest.ini DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_obstacle_cloud_filter test/test_obstacle_cloud_filter.cpp)
  if(TARGET test_obstacle_cloud_filter)
    target_compile_features(test_obstacle_cloud_filter PUBLIC cxx_std_17)
    target_include_directories(test_obstacle_cloud_filter PRIVATE ${PCL_INCLUDE_DIRS})
    ament_target_dependencies(test_obstacle_cloud_filter sensor_msgs)
    target_link_libraries(test_obstacle_cloud_filter
      obstacle_cloud_filter_pipeline
      ${PCL_LIBRARIES}
    )
  endif()

  ament_add_gtest(test_go2_bridge_state test/test_go2_bridge_state.cpp)
  if(TARGET test_go2_bridge_state)
    target_compile_features(test_go2_bridge_state PUBLIC cxx_std_17)
    target_include_directories(test_go2_bridge_state
      PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
  endif()

  ament_add_gtest(test_componentwise_min_speed test/test_componentwise_min_speed.cpp)
  if(TARGET test_componentwise_min_speed)
    target_compile_features(test_componentwise_min_speed PUBLIC cxx_std_17)
    target_include_directories(test_componentwise_min_speed
      PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    ament_target_dependencies(test_componentwise_min_speed
      dwb_core
      nav_2d_msgs
      pluginlib
    )
  endif()

  ament_add_pytest_test(test_arm_image_recorder test/test_arm_image_recorder.py)
  ament_add_pytest_test(test_go2_costmap_rviz test/test_go2_costmap_rviz.py)
  ament_add_pytest_test(test_easy_case_config test/test_easy_case_config.py)
  ament_add_pytest_test(test_exploration test/test_exploration.py)
  ament_add_pytest_test(test_geometry test/test_geometry.py)
  ament_add_pytest_test(test_latest_frame_worker test/test_latest_frame_worker.py)
  ament_add_pytest_test(test_navigation_gate test/test_navigation_gate.py)
  ament_add_pytest_test(test_raw_depth_probe test/test_raw_depth_probe.py)
  ament_add_pytest_test(test_pipeline test/test_pipeline.py)
  ament_add_pytest_test(test_vlm_client test/test_vlm_client.py)
  ament_add_pytest_test(test_obstacle_chain_config test/test_obstacle_chain_config.py)
  ament_add_pytest_test(test_obstacle_cloud_filter_runtime test/test_obstacle_cloud_filter_runtime.py TIMEOUT 20)
  ament_add_pytest_test(test_go2_profile test/test_go2_profile.py)
  ament_add_pytest_test(test_go2_preflight test/test_go2_preflight.py)
  ament_add_pytest_test(test_go2_readiness test/test_go2_readiness.py)
  ament_add_pytest_test(test_go2_bridge_config test/test_go2_bridge_config.py)
  if(TARGET twist_to_go2_sport_bridge)
    ament_add_pytest_test(test_go2_bridge_runtime test/test_go2_bridge_runtime.py TIMEOUT 20)
  endif()
  if(TARGET lowstate_imu_adapter)
    ament_add_pytest_test(test_lowstate_imu_adapter_runtime test/test_lowstate_imu_adapter_runtime.py TIMEOUT 20)
  endif()
  ament_add_pytest_test(test_go2_operational_config test/test_go2_operational_config.py)
  ament_add_pytest_test(test_go2_offline_tools test/test_go2_offline_tools.py)
  ament_add_pytest_test(test_go2_docs test/test_go2_docs.py)
  ament_add_pytest_test(test_vlmnav_go_scripts test/test_vlmnav_go_scripts.py)
  ament_add_pytest_test(test_hesai_converter_integration test/test_hesai_converter_integration.py)
  ament_add_pytest_test(test_hesai_converter_measurement test/test_hesai_converter_measurement.py)
endif()

ament_package()
