add_library(hubozmp zmpwalkgenerator.cpp footprint.cpp gait-timer.cpp swing.cpp)


add_gui_app(zmpdemo zmpdemo.cpp)
add_executable(swingtest swing-test.cpp)
add_executable(footprintdemo footprint-example.cpp)
add_executable(gait-timer-test gait-timer-test.cpp)


set(zmpdemo_libs hubozmp hubomz mzcommon ${OPENGL_LIBRARY} ${GLUT_LIBRARY} expat)


if (${HAVE_HUBO_ACH})
set(zmpdemo_libs ${zmpdemo_libs} ach)
endif()

target_link_libraries(zmpdemo ${zmpdemo_libs})
target_link_libraries(footprintdemo ${zmpdemo_libs})
target_link_libraries(swingtest ${zmpdemo_libs})
target_link_libraries(gait-timer-test ${zmpdemo_libs})
