# 
#  Copyright (C) 2016 Cogimon
#  Author: Enrico Mingo Hoffman
#  email: enrico.mingo@iit.it
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>
#

cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)
include(ExternalProject)
project(rtt_coman)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-typekit)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos-RTT. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

set (Eigen_INCLUDE_DIRS "/usr/include/eigen3")
find_package(orocos_kdl REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(srdfdom_advr REQUIRED)
find_package(XBotCoreModel REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(TINYXML REQUIRED tinyxml)
if(NOT TINYXML_INCLUDE_DIR)
        find_path("/usr/include" tinyxml.h)
        find_library(TINYXML_LIBRARY NAMES tinyxml)
        set(TINYXML_LIBRARIES ${TINYXML_LIBRARY})
    set(TINYXML_INCLUDE_DIRS ${TINYXML_INCLUDE_DIR})
    set(TINYXML_LIBRARY_DIR "/usr/lib/x86_64-linux-gnu")
endif()
message(STATUS "TINYXML_VERSION: ${TINYXML_VERSION}")
include_directories(BEFORE SYSTEM ${TINYXML_INCLUDE_DIRS})
link_directories(${TINYXML_LIBRARY_DIR})
message(STATUS "TINYXML_LIBRARIES: ${TINYXML_LIBRARIES}")
message(STATUS "TINYXML_LIBRARY_DIR: ${TINYXML_LIBRARY_DIR}")
message(STATUS "TINYXML_INCLUDE_DIR: ${TINYXML_INCLUDE_DIRS}")
message(STATUS "TINYXML_LIBRARY: ${TINYXML_LIBRARY}")

include_directories(
                  include
                  ${Boost_INCLUDE_DIR}
                  ${Eigen_INCLUDE_DIRS}
                  ${orocos_kdl_INCLUDE_DIRS}
                  ${USE_OROCOS_INCLUDE_DIRS}
                  ${TINYXML_INCLUDE_DIRS}
                  ${srdfdom_advr_INCLUDE_DIRS}
                  ${XBotCoreModel_INCLUDE_DIRS}
)

file(GLOB_RECURSE rtt_coman_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h)

orocos_component(   ${PROJECT_NAME} 
                    src/rtt_coman.cpp 
                    ${rtt_coman_INCLUDES})
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)


if($ENV{OROCOS_TARGET} STREQUAL "xenomai")
target_link_libraries(${PROJECT_NAME}
   ${USE_OROCOS_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${OROCOS-RTT_LIBRARIES}
   ${Boost_LIBRARIES}
   ${TINYXML_LIBRARIES}
   ${srdfdom_advr_LIBRARIES}
   ${XBotCoreModel_LIBRARIES}
)
else()
target_link_libraries(${PROJECT_NAME}
   ${USE_OROCOS_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${OROCOS-RTT_LIBRARIES}
   ${Boost_LIBRARIES}
   ${TINYXML_LIBRARIES}
   ${srdfdom_advr_LIBRARIES}
   ${XBotCoreModel_LIBRARIES}
)
endif() 


install (TARGETS
         LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
orocos_generate_package(INCLUDE_DIRS include)
