include_directories("./")
include_directories("./Controllers")
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../third-party")
include_directories(SYSTEM "../../third-party/qpOASES/include")
include_directories("../../third-party/ParamHandler")


include_directories("../../lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("/usr/local/include/eigen3")   # eigen3 includes
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

file(GLOB_RECURSE sources
"./*.cpp"
"FSM_States/*.cpp" 
"Controllers/convexMPC/*.cpp")

add_subdirectory(Controllers/WBC)
add_subdirectory(Controllers/WBC_Ctrl)

add_executable(mit_ctrl ${sources} MIT_Controller.cpp main.cpp)
#target_link_libraries(mit_ctrl -L${MKLROOT}/lib/intel64 -Wl,--no-as-needed -lmkl_sequential -lmkl_core -lpthread -lm -ldl)
target_link_libraries(mit_ctrl gamepad robot biomimetics)
target_link_libraries(mit_ctrl dynacore_param_handler qpOASES)
target_link_libraries(mit_ctrl Goldfarb_Optimizer osqp)
target_link_libraries(mit_ctrl WBC_Ctrl)

