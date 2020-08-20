FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)


add_library (WBC_Ctrl SHARED ${headers} ${sources} )
target_link_libraries (WBC_Ctrl biomimetics lcm)
target_link_libraries (WBC_Ctrl WBIC dynacore_param_handler)
#target_link_libraries (WBC_Ctrl  -L${MKLROOT}/lib/intel64 -Wl,--no-as-needed -lmkl_sequential -lmkl_core -lpthread -lm -ldl)


