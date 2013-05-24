cmake_minimum_required (VERSION 2.8)
project (flashtool)

# External dependencies retrieval
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules")

# Make sure warning comes
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-sign-compare -Istandalone")

IF(NOT CMAKE_CROSSCOMPILING)
    SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
ENDIF(NOT CMAKE_CROSSCOMPILING)

# Versioning
set (flashtool_VERSION_MAJOR 0)
set (flashtool_VERSION_MINOR 0)
set (flashtool_VERSION_PATCH 1)

include_directories ("${PROJECT_BINARY_DIR}")

# Include the configured file
#include_directories ("${CMAKE_SOURCE_DIR}/../../platform-phyCARD-Omap/build-target/linux-2.6.33.7/include")
include_directories ("${CMAKE_SOURCE_DIR}/standalone")

# Define Executable Output Directory
set (EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# Include our configuration header
include_directories(${CMAKE_SOURCE_DIR})

# Build the different applications
#add_subdirectory (src)

add_executable(flashtool flashtool.c bchtool.c hammingromcode.c genecc.c genecc.h debug.h)
target_link_libraries(flashtool m)

INSTALL(TARGETS flashtool
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# Create installer for the current platform
set(CPACK_PACKAGE_NAME "flashtool")
set(CPACK_PACKAGE_VENDOR "Fr. Sauter AG")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "flashtool - write to the nand flash using bch ecc encoding")
set(CPACK_PACKAGE_VERSION "${flashtool_VERSION_MAJOR}.${flashtool_VERSION_MINOR}.${flashtool_VERSION_PATCH}")
set(CPACK_PACKAGE_VERSION_MAJOR "${flashtool_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${flashtool_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${flashtool_VERSION_PATCH}") 
set(CPACK_PACKAGE_INSTALL_DIRECTORY "flashtool")
include(CPack)
