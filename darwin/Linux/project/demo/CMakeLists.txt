# swatbotics/darwin/Linux/Project/demo/CMakeLists.txt

add_executable(demo VisionMode.cpp StatusCheck.cpp main.cpp)
target_link_libraries(demo ${DARWIN_LIBS})

