#////////////////////////////////////////////////////////////////////////////
#//
#//  This file is part of RTIMULib
#//
#//  Copyright (c) 2014-2015, richards-tech
#//
#//  Permission is hereby granted, free of charge, to any person obtaining a copy of
#//  this software and associated documentation files (the "Software"), to deal in
#//  the Software without restriction, including without limitation the rights to use,
#//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
#//  Software, and to permit persons to whom the Software is furnished to do so,
#//  subject to the following conditions:
#//
#//  The above copyright notice and this permission notice shall be included in all
#//  copies or substantial portions of the Software.
#//
#//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
#//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
#//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
#//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#// The cmake support was based on work by Moritz Fischer at ettus.com.
#// Original copyright notice:
#
# Copyright 2014 Ettus Research LLC
#

########################################################################
IF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    MESSAGE(FATAL_ERROR "Prevented in-tree built. This is bad practice.")
ENDIF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

########################################################################
# Project setup
########################################################################
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.9)
PROJECT(RTIMULib CXX)
ENABLE_TESTING()

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE(${CMAKE_CURRENT_SOURCE_DIR}/../RTIMULibVersion.txt)

FILE(GLOB LIBRTIMU_SRCS
    "*.cpp"
    "*.h"
    "IMUDrivers/*.cpp"
    "IMUDrivers/*.h"
)

SET(RTMATH_USE_DOUBLE false CACHE BOOL "Use doubles instead of floats internally")

IF(WIN32)
    ADD_LIBRARY(RTIMULib STATIC ${LIBRTIMU_SRCS})
ENDIF(WIN32)

IF(UNIX)
    ADD_LIBRARY(RTIMULib SHARED ${LIBRTIMU_SRCS})
    SET_PROPERTY(TARGET RTIMULib PROPERTY VERSION ${RTIMULIB_VERSION})
    SET_PROPERTY(TARGET RTIMULib PROPERTY SOVERSION ${RTIMULIB_VERSION_MAJOR})
    INSTALL(TARGETS RTIMULib DESTINATION lib)
    INSTALL(DIRECTORY . DESTINATION include FILES_MATCHING PATTERN "*.h")
ENDIF(UNIX)

# Allow access to third_party/*
INCLUDE_DIRECTORIES(..)
IF(RTMATH_USE_DOUBLE)
    target_compile_definitions(RTIMULib PUBLIC RTMATH_USE_DOUBLE=1)
ENDIF(RTMATH_USE_DOUBLE)

find_package(LibFTDI1)
if (LIBFTDI_FOUND)
    find_package(LibFTDISPI REQUIRED)

    target_compile_definitions(RTIMULib PRIVATE RTHAL_LIBFTDI=1)
    target_link_libraries(RTIMULib ${LIBFTDI_LIBRARY} ${LIBFTDISPI_LIBRARY})
else()
    find_library(MPSSE MPSSE ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libMPSSE)
    if(${MPSSE} STREQUAL "MPSSE-NOTFOUND")
        message(FATAL_ERROR "Couldn't find an FTDI (libftdi/libmpsse) library")
    else()
        target_compile_definitions(RTIMULib PRIVATE RTHAL_MPSSE=1)
        target_link_libraries(RTIMULib ${MPSSE})
    endif()
endif()
