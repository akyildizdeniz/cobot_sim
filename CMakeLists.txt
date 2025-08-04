cmake_minimum_required(VERSION 3.5)
project(cobot_sim)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SpeedState.msg"
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install resource file
install(
  FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# Install Python modules
install(
  DIRECTORY cobot_sim/
  DESTINATION lib/python3.12/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install Python scripts as executables individually with proper names
install(PROGRAMS cobot_sim/nodes/visualizer_node.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME visualizer_node)

install(PROGRAMS cobot_sim/nodes/state_logger_node.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME state_logger_node)

install(PROGRAMS cobot_sim/nodes/proximity_sensor_node.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME proximity_sensor_node)

install(PROGRAMS cobot_sim/nodes/emergency_stop_monitor_node.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME emergency_stop_monitor_node)

install(PROGRAMS cobot_sim/nodes/speed_controller_node.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME speed_controller_node)

install(PROGRAMS cobot_sim/nodes/robot_motion_control_node.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME robot_motion_control_node)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  
  # Add the working integration test
  ament_add_pytest_test(test_integration 
    tests/test_integration.py
    TIMEOUT 120
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    APPEND_ENV PYTHONPATH=${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages
  )

  ament_add_pytest_test(test_unit 
    tests/test_unit.py
    TIMEOUT 120
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    APPEND_ENV PYTHONPATH=${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages
  )

  # Standard linting
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()


