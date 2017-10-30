cmake_minimum_required(VERSION 2.8.9 FATAL_ERROR)

########################################################################
IF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    MESSAGE(FATAL_ERROR "Prevented in-tree built. This is bad practice.")
ENDIF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

project(libftdispi)

set(LIBFTDI1_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libftdi1-1.4/cmake/)
find_package(LibFTDI1 REQUIRED)

include_directories(${LIBFTDI_INCLUDE_DIRS})

option(BUILD_EXAMPLES "Build examples" ON)
option(STATIC_LIBS "Build static libraries" ON)

FILE(GLOB LIBFTDISPI_SOURCES
    "src/ftdispi.c"
)

FILE(GLOB LIBFTDISPI_INCLUDES
    "src/ftdispi.h"
)

add_library(ftdispi SHARED ${LIBFTDISPI_SOURCES} ${LIBFTDISPI_INCLUDES})
target_link_libraries(ftdispi ${LIBFTDI_LIBRARY})

install (FILES ${LIBFTDISPI_INCLUDES}
    DESTINATION include/${PROJECT_NAME}
    COMPONENT headers
)

INSTALL(TARGETS ftdispi
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

if (STATIC_LIBS)
    add_library(ftdispi-static STATIC ${LIBFTDISPI_SOURCES} ${LIBFTDISPI_INCLUDES})
    target_link_libraries(ftdispi-static ${LIBFTDI_LIBRARIES})
    set_target_properties(ftdispi-static PROPERTIES OUTPUT_NAME "ftdispi")
    set_target_properties(ftdispi-static PROPERTIES CLEAN_DIRECT_OUTPUT 1)

    INSTALL(TARGETS ftdispi-static
        ARCHIVE DESTINATION lib
        COMPONENT staticlibs
    )
endif ()

if (BUILD_EXAMPLES)
    add_executable(spitest src/spitest.c)
    target_link_libraries(spitest ftdispi)
endif ()

# Install CMake configurations

# config script install path
if ( NOT DEFINED LIBFTDISPI_CMAKE_CONFIG_DIR )
    set (LIBFTDISPI_CMAKE_CONFIG_DIR lib/cmake/libftdispi)
endif ()

include(CMakePackageConfigHelpers)
configure_package_config_file (
    cmake/LibFTDISPIConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/LibFTDISPIConfig.cmake
    INSTALL_DESTINATION ${LIBFTDISPI_CMAKE_CONFIG_DIR}
#   PATH_VARS
#     LIBFTDI_USE_FILE
#     LIBFTDI_ROOT_DIR
#     LIBFTDI_INCLUDE_DIR
#     LIBFTDI_INCLUDE_DIRS
#     LIBFTDI_LIBRARY_DIRS
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

install (FILES 
    ${CMAKE_CURRENT_BINARY_DIR}/LibFTDISPIConfig.cmake
    DESTINATION ${LIBFTDISPI_CMAKE_CONFIG_DIR}
)