# swatbotics/darwin/Linux/CMakeLists.txt

include_directories(../Framework/include)
include_directories(include)

add_subdirectory(build)
set(DARWIN_LIBS darwin pthread jpeg rt)

add_subdirectory(project)
