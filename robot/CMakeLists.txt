include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

include_directories("include/")
include_directories("../common/include/")
include_directories("../common/include/Math")
include_directories("../third-party")
include_directories("../third-party/ParamHandler")
include_directories("../third-party/lord_imu/Include")

include_directories("../lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes

file(GLOB sources "src/*.cpp" "src/rt/*.c"  "src/rt/*.cpp")

add_library(robot SHARED ${sources})

target_link_libraries(robot gamepad biomimetics pthread lcm inih dynacore_param_handler lord_imu)
target_link_libraries(robot rt)
#target_link_libraries(robot  -L${MKLROOT}/lib/intel64 -Wl,--no-as-needed -lmkl_intel_ilp64 -lmkl_sequential -lmkl_core -lpthread -lm -ldl)
