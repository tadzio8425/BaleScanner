cmake_minimum_required(VERSION 3.8)
project(bale_scanner)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TriggerStatic.srv"
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
