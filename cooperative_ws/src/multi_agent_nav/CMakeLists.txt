cmake_minimum_required(VERSION 3.8)
project(multi_agent_nav)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(COOPERATIVE_RUNTIME_PYTHON "${Python3_EXECUTABLE}" CACHE FILEPATH "Python used by task nodes and pytest (same CPython ABI as ROS)")
ament_python_install_package(${PROJECT_NAME})
foreach(entry agent_node coordinator_node start_task calibration)
  set(ENTRY_MODULE ${entry})
  configure_file(scripts/entry.in ${CMAKE_CURRENT_BINARY_DIR}/multi_agent_${entry} @ONLY)
  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/multi_agent_${entry} DESTINATION lib/${PROJECT_NAME})
endforeach()
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(domain_bridge REQUIRED)
  add_executable(check_bridge_routes tests/check_bridge_routes.cpp)
  target_link_libraries(check_bridge_routes domain_bridge::domain_bridge)
  ament_add_test(bridge_routes_parser
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND $<TARGET_FILE:check_bridge_routes>
      ${CMAKE_CURRENT_SOURCE_DIR}/config/bridge_go2_pending.yaml
      ${CMAKE_CURRENT_SOURCE_DIR}/config/bridge_ranger_pending.yaml)
  ament_has_pytest(HAS_COOPERATIVE_PYTEST PYTHON_EXECUTABLE "${COOPERATIVE_RUNTIME_PYTHON}")
  if(NOT HAS_COOPERATIVE_PYTEST)
    message(FATAL_ERROR "Cooperative pytest unavailable; build with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
  endif()
  ament_add_pytest_test(cooperative_logic tests
    TIMEOUT 120
    PYTHON_EXECUTABLE "${COOPERATIVE_RUNTIME_PYTHON}"
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=31 ROS_LOCALHOST_ONLY=1
    APPEND_ENV
      "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/../../..")
endif()
ament_package()
