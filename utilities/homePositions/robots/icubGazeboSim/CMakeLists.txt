set(robotname icubGazeboSim)

file(GLOB ini ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)

yarp_install(FILES ${ini} DESTINATION ${YARP_ROBOTS_INSTALL_DIR}/${robotname})
