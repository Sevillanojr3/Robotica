cmake_minimum_required(VERSION 3.0.2)
project(mi_robot_pkg)

## Encuentra los paquetes necesarios para construir este paquete
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

## Declara los archivos de mensajes
add_message_files(
  FILES
  HandData.msg
)

## Genera mensajes con dependencias
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declara el paquete de catkin y sus dependencias
catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs geometry_msgs
)

## Incluye directorios necesarios
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Opcional: Configuración de compilación en C++ (si decides usar C++ más adelante)
# add_compile_options(-std=c++11)