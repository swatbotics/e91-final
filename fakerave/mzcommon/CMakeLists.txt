# CMakeLists.txt for hubomz/mzcommon

set(mzcommon_srcs
  mersenne.c
  gauss.c
  GlCamera.cpp
  MzGlutApp.cpp
  TinyDom.cpp
  TriMesh3.cpp
  glstuff.cpp)

if (QT_FOUND)
  set(mzcommon_srcs ${mzcommon_srcs} MzGlWidget.cpp)
endif()

add_library( mzcommon ${mzcommon_srcs} )

add_executable(testeigen testeigen.cpp)
target_link_libraries(testeigen mzcommon)

add_executable(testwrl testwrl.cpp)
target_link_libraries(testwrl mzcommon ${OPENGL_LIBRARY})



