project(SensorLogger)

cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(HAL REQUIRED)
find_package(Calibu)
find_package(OpenCV)
find_package(Protobuf)

if(Calibu_FOUND AND OpenCV_FOUND AND Protobuf_FOUND)

option(BUILD_SensorLogger "Build the SensorLogger application." ON)

if(BUILD_SensorLogger)


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
endif()
