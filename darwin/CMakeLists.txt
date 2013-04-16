# swatbotics/darwin/CMakeLists.txt

if (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
add_subdirectory(Linux)
endif (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

