get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

# Find HAL
find_package(HAL REQUIRED)
include_directories(${HAL_INCLUDE_DIRS})

add_executable( ${ProjectId} main.cpp)
target_link_libraries(${ProjectId}
                      ${HAL_LIBRARIES}
                      )

