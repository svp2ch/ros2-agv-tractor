cmake_minimum_required(VERSION 3.8)
project(agv_description)

find_package(ament_cmake REQUIRED)

# Устанавливаем все папки с ресурсами в share/<package>/
install(
  DIRECTORY urdf meshes launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
