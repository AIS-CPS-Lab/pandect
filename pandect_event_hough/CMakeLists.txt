cmake_minimum_required(VERSION 3.8)
project(pandect_event_hough)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

set(TARGET_NAME pandect_event_hough_node_component)

add_library(${TARGET_NAME} SHARED src/pandect_event_hough_node.cpp)

rclcpp_components_register_node(${TARGET_NAME}
  PLUGIN "pandect_event_hough::HoughTransformNode"
  EXECUTABLE pandect_event_hough_node
)

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS} 
  ${cv_bridge_INCLUDE_DIRS}
)
target_compile_features(${TARGET_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${TARGET_NAME}
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  event_camera_codecs
  event_camera_msgs
)

install(TARGETS ${TARGET_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
