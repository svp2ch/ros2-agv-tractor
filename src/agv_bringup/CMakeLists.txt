cmake_minimum_required(VERSION 3.8)
project(agv_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Папку config/ устанавливаем только если она существует
# (сейчас она пустая/отсутствует, конфиги добавятся позже)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config/")
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

ament_package()
