project(ModeViewer)

cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(Calibu)
find_package(Pangolin 0.1)
find_package(OpenCV)
find_package(Protobuf)
find_package(SceneGraph)


if(Calibu_FOUND AND Pangolin_FOUND AND OpenCV_FOUND AND Protobuf_FOUND AND SceneGraph_FOUND )

option(BUILD_ModelViewer "Build the ModelViewer application." ON)

if(BUILD_ModelViewer)


if(APPLE AND NOT BUILD_SHARED_LIBS)
  set(LINK_WHOLE_ARCHIVE_FLAG -Wl,-all_load)
elseif(ANDROID)
  set(LINK_WHOLE_ARCHIVE_FLAG -Wl,--whole-archive)
  set(NO_LINK_WHOLE_ARCHIVE_FLAG -Wl,--no-whole-archive)
endif()

  add_executable( ModelViewer main.cpp )

  include_directories( ${Protobuf_INCLUDE_DIR} )
  include_directories( ${Calibu_INCLUDE_DIRS} )
  include_directories( ${SceneGraph_INCLUDE_DIRS} )
  include_directories( ${Pangolin_INCLUDE_DIRS} )
  target_link_libraries(ModelViewer
    ${Pangolin_LIBRARIES}
    ${SceneGraph_LIBRARIES}
    ${LINK_WHOLE_ARCHIVE_FLAG}
    ${Calibu_LIBRARIES}
    ${NO_LINK_WHOLE_ARCHIVE_FLAG}
    ${Protobuf_LIBRARY}
     dl
     )

endif()

endif()

