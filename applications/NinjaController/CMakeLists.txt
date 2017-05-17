project(NinjaController)

cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

option(BUILD_NinjaController "This controls the ninjaV3 with visual odometry feedback for the controller." OFF)

find_package(HAL REQUIRED)
#find_package(spirit)

set( _INCLUDE_DIRS
  ${HAL_INCLUDE_DIRS}
#  ${spirit_INCLUDE_DIRS}
  )

set( _LIBRARIES
  ${HAL_LIBRARIES}
#  ${spirit_LIBRARY}
  )

IF(BUILD_NinjaController)
  add_executable( ${PROJECT_NAME} main.cpp )
  include_directories( ${_INCLUDE_DIRS} )
  target_link_libraries(${PROJECT_NAME} ${_LIBRARIES})
endif()

