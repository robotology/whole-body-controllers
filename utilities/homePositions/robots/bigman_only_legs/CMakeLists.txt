set(robotname bigman_only_legs)

file(GLOB ini ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)

yarp_install(FILES ${ini} DESTINATION ${WBC_ROBOTS_INSTALL_DIR}/${robotname})
