set(robotname iCubGazeboV2_5_plus)

file(GLOB ini ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)

yarp_install(FILES ${ini} DESTINATION ${WBC_ROBOTS_INSTALL_DIR}/${robotname})
