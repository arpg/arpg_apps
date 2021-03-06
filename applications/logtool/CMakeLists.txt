project(logtool)
cmake_minimum_required(VERSION 2.8)

find_package(HAL REQUIRED)
find_package(GFlags)
find_package(GLog)
find_package(OpenCV)
find_package(Protobuf)

if( GFlags_FOUND AND GLog_FOUND AND OpenCV_FOUND AND Protobuf_FOUND )

option(BUILD_LOGTOOL "Build the Logtool application." ON)

if(BUILD_LOGTOOL)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


#install_package(
#    PKG_NAME logtool
#    VERSION 0.1
#    DESTINATION ${CMAKE_INSTALL_PREFIX}
#    INCLUDE_DIRS ${HAL_INCLUDES}
#    LINK_LIBS ${HAL_LIBS} messages
#    )

add_executable( logtool logtool.cpp )
include_directories(
  ${GFLAGS_INCLUDE_DIR}
  ${GLog_INCLUDE_DIRS}
  ${OpenCV2_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/HAL
  ${HAL_INCLUDE_DIRS}
  ${Calibu_INCLUDE_DIRS}
  )
target_link_libraries( logtool
    ${HAL_LIBRARIES}
    ${Calibu_LIBRARIES}
    ${GFLAGS_LIBRARIES}
    ${PROTOBUF_LIBRARY}
)

#def_executable(logtool
#  SOURCES logtool.cpp
#  DEPENDS messages
#  LINK_LIBS ${GFLAGS_LIBRARIES} ${GLog_LIBRARIES} ${OpenCV2_LIBRARIES} ${HAL_LIBRARIES}
#  CONDITIONS GFLAGS_FOUND GLog_FOUND OpenCV2_FOUND
#  PACKAGE messages
#  )

endif()
endif()
