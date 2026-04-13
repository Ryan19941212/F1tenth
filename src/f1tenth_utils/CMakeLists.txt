cmake_minimum_required(VERSION 3.5)
project(f1tenth_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/sim_drive_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
