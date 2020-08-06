cmake_minimum_required(VERSION 3.5.1)

# Setup the project.
# Its name is defined to be the controller directory name.
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})

#C++ 11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Get C or C++ sources in the current directory (only).
file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})

# Link with the Webots controller library.
link_directories($ENV{WEBOTS_HOME}/lib/controller)

#include
include_directories(./lcm_types)
include_directories(/usr/local/include/)         # lcm includes
include_directories(${PROJECT_SOURCE_DIR})
include_directories($ENV{WEBOTS_HOME}/include/controller/c)
include_directories($ENV{WEBOTS_HOME}/include/controller/cpp)

set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})

# Setup the target executable.
add_executable(${PROJECT} ${SOURCES})
#link
target_link_libraries(${PROJECT} lcm)
target_link_libraries(${PROJECT} ${LIBRARIES})
target_link_libraries(${PROJECT} -lpthread -lm)

# Copy the target executable at the right location.
add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR}
)
