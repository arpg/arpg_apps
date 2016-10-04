project(PoseToNode)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(HAL REQUIRED)
find_package(Node QUIET)
find_package(GFlags)
find_package(GLog)

if(Node_FOUND AND GFLags_FOUND AND GLog_FOUND)

option(BUILD_PoseToNode "Build the PoseToNode application." OFF)

if(BUILD_PoseToNode)


  if(APPLE AND NOT BUILD_SHARED_LIBS)
    set(LINK_WHOLE_ARCHIVE_FLAG -Wl,-all_load)
  elseif(ANDROID)
    set(LINK_WHOLE_ARCHIVE_FLAG -Wl,--whole-archive)
    set(NO_LINK_WHOLE_ARCHIVE_FLAG -Wl,--no-whole-archive)
  endif()

  if(BUILD_PoseToNode)
      add_executable( ${PROJECT_NAME} main.cpp )

      include_directories(${GFLAGS_INCLUDE_DIRS} ${HAL_INCLUDE_DIRS} ${Node_INCLUDE_DIRS}
          ${GLOG_INCLUDE_DIRS})
      target_link_libraries( ${PROJECT_NAME}
          ${GFLAGS_LIBRARIES}
          ${LINK_WHOLE_ARCHIVE_FLAG}
          ${HAL_LIBRARIES}
          ${NO_LINK_WHOLE_ARCHIVE_FLAG}
          ${GLOG_LIBRARIES}
          ${Node_LIBRARIES}
          dl
          )
  endif()
endif()
endif()
