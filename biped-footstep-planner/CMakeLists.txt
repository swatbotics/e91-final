cmake_minimum_required(VERSION 2.8)

project(BIPED_FOOTSTEP_PLANNER)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

include_directories(include)

if(APPLE)
  include_directories(/opt/local/include) # MacPorts
  find_library(OPENGL_LIBRARY OpenGL)
else(APPLE)
  find_library(OPENGL_LIBRARY GL)
  find_library(GLU_LIBRARY GLU)
  set(OPENGL_LIBRARY ${OPENGL_LIBRARY} ${GLU_LIBRARY})
endif(APPLE)

find_library(EXPAT_LIBRARY expat)
find_library(GLUT_LIBRARY glut)
find_library(PNG_LIBRARY png)

set(CMAKE_C_FLAGS "-Wall -g")
set(CMAKE_CXX_FLAGS "-Wall -g")

if (APPLE)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wsign-compare")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -Wsign-compare")
endif (APPLE)

set(CMAKE_C_FLAGS_DEBUG "-O")
set(CMAKE_CXX_FLAGS_DEBUG "-O")

set(CMAKE_C_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

macro(add_gui_app name)
if (APPLE)
  add_executable(${name} MACOSX_BUNDLE ${ARGN})
else (APPLE)
  add_executable(${name} ${ARGN})
endif (APPLE)
endmacro(add_gui_app)

add_library(bplanner src/biped.cpp src/biped_checker.cpp src/bipedSearch.cpp)

add_executable(bipedPlan src/bipedPlan.cpp)
target_link_libraries(bipedPlan bplanner ${PNG_LIBRARY})

add_gui_app( bipedDemo src/bipedDemo.cpp)
target_link_libraries( bipedDemo bplanner ${GLUT_LIBRARY} ${OPENGL_LIBRARY} 
  ${PNG_LIBRARY} ${EXPAT_LIBRARY} )
