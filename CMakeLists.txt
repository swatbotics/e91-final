# swatbotics/CMakeLists.txt

cmake_minimum_required(VERSION 2.8)

project(e91-final)

include(FindPkgConfig)

#pkg_check_modules(CAIRO REQUIRED cairo)
#pkg_search_module(OPENCV REQUIRED opencv>=2.3 opencv-2.3.1)
#pkg_check_modules(SDL sdl)

if (APPLE)
  find_library(OPENGL_LIBRARY OpenGL)
  find_library(GLUT_LIBRARY Glut)
  set(OPENGL_LIBRARIES ${OPENGL_LIBRARY} )
else ()
  find_library(OPENGL_LIBRARY GL)
  find_library(GLU_LIBRARY GLU)
  find_library(GLUT_LIBRARY glut)
  set(OPENGL_LIBRARIES ${OPENGL_LIBRARY} ${GLU_LIBRARY})
endif ()

# Do parallel (out-of-source) builds
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

# If build type not specified, make it release
if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif ()

set(CMAKE_C_FLAGS "-Wall -g")
set(CMAKE_CXX_FLAGS "-Wall -g")

set(CMAKE_C_FLAGS_DEBUG "-O")
set(CMAKE_CXX_FLAGS_DEBUG "-O")

set(CMAKE_C_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# Put in order so that april and darwin code can link against gflags, and
# so darwin code can link against the april library.
#add_subdirectory(gflags)
#add_subdirectory(libconfig)
#add_subdirectory(april)
add_subdirectory(darwin)

#if (NOT APPLE)
#add_subdirectory(ptpd)
#endif (NOT APPLE)

