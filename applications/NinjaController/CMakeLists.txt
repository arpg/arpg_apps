project(NinjaController)

cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")

option(BUILD_NinjaController "This controls the ninjaV3 with visual odometry feedback for the controller." OFF)

IF(BUILD_NinjaController)
  find_package(HAL REQUIRED)
  find_package(spirit)
  find_package(Pangolin)

  set( _INCLUDE_DIRS
    ${HAL_INCLUDE_DIRS}
    ${spirit_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    )

  set( _LIBRARIES
    ${HAL_LIBRARIES}
    ${spirit_LIBRARIES}
    ${Pangolin_LIBRARIES}
    )
  add_executable( NinjaController main.cpp spPID.h spPID.cpp)
  include_directories( ${_INCLUDE_DIRS} )
  target_link_libraries(NinjaController
    ${_LIBRARIES}
    )
endif()

