cmake_minimum_required(VERSION 3.5)
project(sim)

SET(CMAKE_INCLUDE_CURRENT_DIR ON)

execute_process(COMMAND ${CMAKE_SOURCE_DIR}/scripts/find_qt_path.sh OUTPUT_VARIABLE CMAKE_PREFIX_PATH_1)
SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH_1})
message("CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

if(POLICY CMP0071)
  cmake_policy(SET CMP0071 OLD)
endif()

# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)
find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)
get_target_property(QtCore_location Qt5::Core LOCATION)
message(STATUS ${QtCore_location})
find_package(Qt5Gui CONFIG REQUIRED)

set(QOBJECT_HEADERS
        include/Graphics3D.h
        include/SimControlPanel.h)

QT5_WRAP_CPP(tst_hdr_moc ${QOBJECT_HEADERS})

include_directories("include/")
include_directories("../common/include/")
include_directories("../lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("../third-party/ParamHandler")
include_directories("../third-party")

include_directories(${CMAKE_BINARY_DIR})

file(GLOB sources "src/*.cpp")

set(OpenGL_GL_PREFERENCE LEGACY)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

add_executable(sim ${sources} ${tst_hdr_moc})
target_link_libraries(sim gamepad biomimetics pthread lcm inih dynacore_param_handler 
    Qt5::Widgets Qt5::Core Qt5::Gui ${QT_LIBRARIES} ${OPENGL_LIBRARIES})

target_link_libraries(sim rt ${GLUT_glut_LIBRARIES})
include (CheckIncludeFileCXX)
#target_link_libraries(sim  -L${MKLROOT}/lib/intel64 -Wl,--no-as-needed -lmkl_intel_ilp64 -lmkl_sequential -lmkl_core -lpthread -lm -ldl)

