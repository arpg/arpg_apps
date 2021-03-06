project(Gamepad)

cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(HAL REQUIRED)
find_package(Protobuf )

if( Protobuf_FOUND )

option(BUILD_OptitrackTest "Build the OptitrackTest application." OFF)

IF(BUILD_OptitrackTest)

include_directories(${PROTOBUF_INCLUDE_DIR})

if(APPLE AND NOT BUILD_SHARED_LIBS)
  set(LINK_WHOLE_ARCHIVE_FLAG -Wl,-all_load)
elseif(ANDROID)
  set(LINK_WHOLE_ARCHIVE_FLAG -Wl,--whole-archive)
  set(NO_LINK_WHOLE_ARCHIVE_FLAG -Wl,--no-whole-archive)
endif()

  add_executable( OptitrackTest main.cpp )

  include_directories( ${HAL_INCLUDE_DIRS} )
  target_link_libraries(OptitrackTest
    ${HAL_LIBRARIES}
    ${PROTOBUF_LIBRARY}
     dl
     )

endif()

endif()
