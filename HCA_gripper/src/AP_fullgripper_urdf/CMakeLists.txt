cmake_minimum_required(VERSION 3.8)
project(AP_fullgripper_urdf)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    meshes
    textures
    urdf
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
