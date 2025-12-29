cmake_minimum_required(VERSION 3.8)
project(rm520n_multi_driver LANGUAGES CXX)

# ---------------- Basics ----------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
include(GNUInstallDirs)

# ---------------- Deps ----------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tod_msgs REQUIRED)

# (필요 시 주석 해제: Boost.Asio(Boost 버전) 사용한다면 Boost::system 링크)
# find_package(Boost QUIET COMPONENTS system)
# find_package(Threads REQUIRED)

# ---------------- Component Library ----------------
add_library(rm520n_multi_driver_component SHARED
  src/serial_interface.cpp
  src/nmea_parser.cpp
  src/at_interface.cpp
  src/rm520n_multi_driver_node.cpp
)

target_include_directories(rm520n_multi_driver_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(rm520n_multi_driver_component
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  tod_msgs
)

# (필요 시 주석 해제)
# if(Boost_FOUND)
#   target_link_libraries(rm520n_multi_driver_component Boost::system)
# endif()
# target_link_libraries(rm520n_multi_driver_component Threads::Threads)

rclcpp_components_register_nodes(
  rm520n_multi_driver_component
  "rm520n_multi_driver::RM520NMultiDriverNode"
)

# ---------------- Standalone Node ----------------
add_executable(rm520n_multi_driver_node
  src/main.cpp
)
target_link_libraries(rm520n_multi_driver_node
  rm520n_multi_driver_component
)
ament_target_dependencies(rm520n_multi_driver_node
  rclcpp
)

# ---------------- Install / Export ----------------
install(TARGETS
  rm520n_multi_driver_component
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(TARGETS
  rm520n_multi_driver_node
  RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  tod_msgs
)

ament_package()
