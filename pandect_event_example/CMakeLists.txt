cmake_minimum_required(VERSION 3.8)
project(pandect_event_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED) # For creating component nodes
find_package(event_camera_msgs REQUIRED) # For event packet messages
find_package(event_camera_codecs REQUIRED) # For event packet decoding

# Add files to the component node
add_library(${PROJECT_NAME}_component SHARED src/pandect_event_example_node.cpp)

# Register the component node
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "pandect_event_example::PandectEventExampleNode"
  EXECUTABLE pandect_event_example_node # This also creates a standalone node
)

# Add include directories for the component
target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}_component PUBLIC c_std_99 cxx_std_17)

# Add dependencies for the component
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  event_camera_msgs
  event_camera_codecs
)

# Install the component library
install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the launch files
# Note: The launch directory is optional, so it won't fail if it doesn't exist
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install the config files
# Note: The config directory is optional, so it won't fail if it doesn't exist
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
