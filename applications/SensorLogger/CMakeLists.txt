project(SensorLogger)

cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

option(BUILD_SensorLogger "Build the SensorLogger application." ON)

if(BUILD_SensorLogger)

find_package(Calibu REQUIRED)
find_package(HAL REQUIRED)
find_package(OpenCV2 REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(${PROTOBUF_INCLUDE_DIR})

if(APPLE AND NOT BUILD_SHARED_LIBS)
  set(LINK_WHOLE_ARCHIVE_FLAG -Wl,-all_load)
elseif(ANDROID)
  set(LINK_WHOLE_ARCHIVE_FLAG -Wl,--whole-archive)
  set(NO_LINK_WHOLE_ARCHIVE_FLAG -Wl,--no-whole-archive)
endif()

add_executable( SensorLogger main.cpp )

  include_directories( ${HAL_INCLUDE_DIRS} )
  include_directories( ${Calibu_INCLUDE_DIRS} )
  target_link_libraries(SensorLogger
    ${LINK_WHOLE_ARCHIVE_FLAG}
    ${HAL_LIBRARIES}
    ${Calibu_LIBRARIES}
    ${NO_LINK_WHOLE_ARCHIVE_FLAG}
    ${PROTOBUF_LIBRARY}
     dl
     )

endif()
