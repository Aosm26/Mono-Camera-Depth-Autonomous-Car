cmake_minimum_required(VERSION 3.5)
project(autonomous_car)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY launch urdf config worlds media
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
