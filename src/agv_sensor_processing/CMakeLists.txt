cmake_minimum_required(VERSION 3.8)
project(agv_sensor_processing)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Устанавливаем Python-пакет (папку agv_sensor_processing/)
ament_python_install_package(${PROJECT_NAME})

# Устанавливаем Python-скрипты как исполняемые узлы ROS 2
install(PROGRAMS
  agv_sensor_processing/sensor_fusion.py
  agv_sensor_processing/lidar_processing.py
  DESTINATION lib/${PROJECT_NAME}
)

# Конфиги и launch-файлы
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
