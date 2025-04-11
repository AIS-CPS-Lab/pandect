cmake_minimum_required(VERSION 3.29)
project(pandect)

#include(FetchContent)
#FetchContent_Declare(
#        openEB
#        GIT_REPOSITORY git@github.com:prophesee-ai/openeb.git
#        GIT_TAG main
#)
#FetchContent_MakeAvailable(openEB)

set(CMAKE_CXX_STANDARD 20)
#find_package(MetavisionSDK COMPONENTS core stream ui REQUIRED)

add_executable(event_camera main.cpp)
