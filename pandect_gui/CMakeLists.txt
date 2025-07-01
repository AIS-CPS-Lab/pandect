cmake_minimum_required(VERSION 3.8)
project(pandect_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)
find_package(event_camera_msgs REQUIRED)

include(FetchContent)
set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)
FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG edd2597344f21cc491f65354849dee1d66c2e427
)
FetchContent_MakeAvailable(threepp)

set(TARGET_NAME pandect_gui)

add_executable(${TARGET_NAME} src/pandect_gui.cpp)
add_executable(${TARGET_NAME}_pc_publsher src/pandect_pc_publisher.cpp)

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(${TARGET_NAME}_pc_publsher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${TARGET_NAME} PUBLIC c_std_99 cxx_std_17)
target_compile_features(${TARGET_NAME}_pc_publsher PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${TARGET_NAME}
  rclcpp
  std_msgs
  event_camera_codecs
)

ament_target_dependencies(${TARGET_NAME}_pc_publsher
  rclcpp
  std_msgs
  event_camera_codecs
  event_camera_msgs
  sensor_msgs
)

target_link_libraries(${TARGET_NAME}
  threepp::threepp
)

install(TARGETS ${TARGET_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${TARGET_NAME}_pc_publsher
  DESTINATION lib/${PROJECT_NAME}
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
