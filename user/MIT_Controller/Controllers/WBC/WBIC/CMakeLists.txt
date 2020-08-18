FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)


add_library(WBIC SHARED ${sources} ${headers})
target_link_libraries(WBIC Goldfarb_Optimizer)
target_link_libraries(WBIC -L${MKLROOT}/lib/intel64 -Wl,--no-as-needed -lmkl_sequential -lmkl_core -lpthread -lm -ldl)
