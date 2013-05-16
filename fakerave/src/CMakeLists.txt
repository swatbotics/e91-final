# CMakeLists.txt for hubomz/src

add_library( fakerave fakerave.cpp fakerave_parse.cpp Biped.cpp Darwin.cpp )

add_gui_app( testrave testrave.cpp )
target_link_libraries( testrave fakerave mzcommon ${GLUT_LIBRARY} 
  ${OPENGL_LIBRARY} expat )
