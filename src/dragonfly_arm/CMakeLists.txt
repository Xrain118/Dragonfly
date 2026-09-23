cmake_minimum_required(VERSION 3.8)
project(dragonfly_arm)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS scripts/demo_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME demo_trajectory
)

if(BUILD_TESTING)
  # The host has user-installed pytest plugins that are newer than Ubuntu's
  # pytest. Keep this package's self-contained tests isolated from them.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_urdf
    test/test_urdf.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
endif()

ament_package()
