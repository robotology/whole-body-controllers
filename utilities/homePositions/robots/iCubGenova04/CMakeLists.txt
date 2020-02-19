set(robotname iCubGenova04)

file(GLOB ini ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)

yarp_install(FILES ${ini} DESTINATION ${WBC_ROBOTS_INSTALL_DIR}/${robotname})
