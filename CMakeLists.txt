# Author: Gabriele Nava
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.11)
project(whole-body-controllers)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(WHOLE-BODY-CONTROLLERS_SHARE_DIR "${CMAKE_INSTALL_PREFIX}/share/whole-body-controllers")

# Fake installation. 
install(CODE "MESSAGE(\"This repository simply acts as a container\")")
