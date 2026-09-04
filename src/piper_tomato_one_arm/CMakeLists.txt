cmake_minimum_required(VERSION 3.22)
project(piper_tomato_one_arm)

find_package(ament_cmake REQUIRED)

install(DIRECTORY src/urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY src/meshes/ DESTINATION share/${PROJECT_NAME}/meshes)

ament_package()
