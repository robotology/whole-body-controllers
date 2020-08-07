################################################################################
#                                                                              #
# Copyright (C) 2018 Fondazione Istitito Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

# Install the Simulink models into the install folder
file(GLOB slx_files "*.slx")
install(FILES ${slx_files} DESTINATION ${CMAKE_INSTALL_PREFIX}/mex/+wbc/simulink)

# Install the MATLAB script into the install folder
file(GLOB m_files "*.m")
install(FILES ${m_files} DESTINATION ${CMAKE_INSTALL_PREFIX}/mex/+wbc/simulink)

################################################################################

# Install the sublibraries related files into the install file

macro(INSTALL_SUBLIBMFILES curdir)
# This macro installs all the matlab files inside the specified folder into the install folder
  file(GLOB children RELATIVE ${curdir} ${curdir}/src/*)
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
    # This applies to the folders inside this directory <sublib_dir>/src are assumed to contain matlab files
      file(GLOB m_files ${curdir}/${child}/*.m)
      install(FILES ${m_files} DESTINATION ${CMAKE_INSTALL_PREFIX}/mex/+wbc)
    endif()
  endforeach()
  # The following applies to the matlab files inside this directory <sublib_dir>/src
  file(GLOB m_files_out ${curdir}/src/*.m)
  install(FILES ${m_files_out} DESTINATION ${CMAKE_INSTALL_PREFIX}/mex/+wbc)
endmacro()

macro(INSTALL_SUBLIBS curdir)
  file(GLOB sublibs_dirs RELATIVE ${curdir} ${curdir}/*)
  foreach(sublib_dir ${sublibs_dirs})
    if(IS_DIRECTORY ${curdir}/${sublib_dir})
      # Install the Simulink models inside each sublibrary folder into the install folder
      file(GLOB sublib_slx_files ${sublib_dir}/*.slx)
      install(FILES ${sublib_slx_files} DESTINATION ${CMAKE_INSTALL_PREFIX}/mex/+wbc/simulink)
      # Install Matlab files inside each sublibrary folder into the install folder
      INSTALL_SUBLIBMFILES(${CMAKE_CURRENT_SOURCE_DIR}/${sublib_dir})
    endif()
  endforeach()
endmacro()

INSTALL_SUBLIBS(${CMAKE_CURRENT_SOURCE_DIR})
