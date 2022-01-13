cmake_minimum_required(VERSION 3.0.2)
project(lib221e)

set(LIB221E_MAJOR_VERSION 0)
set(LIB221E_MINOR_VERSION 1)
set(LIB221E_PATCH_VERSION 0)
set(LIB221E_VERSION
  ${LIB221E_MAJOR_VERSION}.${LIB221E_MINOR_VERSION}.${LIB221E_PATCH_VERSION})

## Project Output Paths
set(CMAKE_BINARY_DIR "${CMAKE_SOURCE_DIR}/build")
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/lib")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})

set(INSTALL_LIB_DIR ${CMAKE_SOURCE_DIR}/lib)
set(INSTALL_BIN_DIR ${CMAKE_SOURCE_DIR}/bin)
set(INSTALL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)

if(WIN32 AND NOT CYGWIN)
   set(DEF_INSTALL_CMAKE_DIR ${CMAKE_SOURCE_DIR}/CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR /lib/cmake/lib221e)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR})

set(PROJECT_SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)
set(PROJECT_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)

include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_INCLUDE_DIR}")
include_directories("${PROJECT_BINARY_DIR}")

#set(Boost_DEBUG 1)
find_package(Boost REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

if(UNIX)
    find_library(GATTLIB NAMES gattlib)
endif()

#Connection

if(UNIX AND NOT APPLE)
    # If Linux, add rt and pthread
    set(rt_LIBRARIES rt)
    set(pthread_LIBRARIES pthread)
endif()

## Sources
set(CONNECTION_SOURCES
    ${PROJECT_SOURCE_DIR}/serial/SerialConnection.cpp
    ${PROJECT_INCLUDE_DIR}/serial/SerialConnection.h
    ${PROJECT_INCLUDE_DIR}/serial/v8stdint.h
)
if(UNIX)
    # If unix
    list(APPEND CONNECTION_SOURCES ${PROJECT_SOURCE_DIR}/serial/impl/SerialConnectionUnix.cpp)
    list(APPEND CONNECTION_SOURCES ${PROJECT_SOURCE_DIR}/serial/impl/list_ports/ListPortsLinux.cpp)
else()
    # If windows
    list(APPEND CONNECTION_SOURCES ${PROJECT_SOURCE_DIR}/serial/impl/SerialConnectionWindows.cpp)
    list(APPEND CONNECTION_SOURCES ${PROJECT_SOURCE_DIR}/serial/impl/list_ports/ListPortsWin.cpp)
endif()

## Add serial library
add_library(Connection ${CONNECTION_SOURCES})
if(UNIX)
    target_link_libraries(Connection rt pthread)
else()
    target_link_libraries(Connection setupapi)
endif()

install(TARGETS Connection
  # IMPORTANT: Add the Core library to the "export-set"
  EXPORT lib221eTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/serial"
    COMPONENT dev)

##Mitch

if(UNIX)
    set(MITCH_V2_INCLUDES	${PROJECT_INCLUDE_DIR}/mitch_v2/MitchV2_HW.h
                            ${PROJECT_INCLUDE_DIR}/mitch_v2/MitchV2_Data.h
                            ${PROJECT_INCLUDE_DIR}/mitch_v2/MitchV2_StreamingData.h
                            ${PROJECT_INCLUDE_DIR}/mitch_v2/MitchV2_SerialConnection.h	
                            ${PROJECT_INCLUDE_DIR}/mitch_v2/MitchV2_BLEConnection.h	
    )

    set(MITCH_V2_SOURCES	${PROJECT_SOURCE_DIR}/mitch_v2/MitchV2_SerialConnection.cpp	
                            ${PROJECT_SOURCE_DIR}/mitch_v2/MitchV2_BLEConnection.cpp
                            ${PROJECT_SOURCE_DIR}/mitch_v2/MitchV2_StreamingData.cpp	
   )
else()
    set(MITCH_V2_INCLUDES	${PROJECT_INCLUDE_DIR}/mitch_v2/MitchV2_HW.h
                            ${PROJECT_INCLUDE_DIR}/mitch_v2/MitchV2_Data.h
                            ${PROJECT_INCLUDE_DIR}/mitch_v2/MitchV2_SerialConnection.h	
   )

    set(MITCH_V2_SOURCES	${PROJECT_SOURCE_DIR}/mitch_v2/MitchV2_SerialConnection.cpp	
   )

    set(MUSE_V2_INCLUDES	${PROJECT_INCLUDE_DIR}/muse_v2/MuseV2_HW.h
                            ${PROJECT_INCLUDE_DIR}/muse_v2/MuseV2_SerialConnection.h	
   )

    set(MUSE_V2_SOURCES	    ${PROJECT_SOURCE_DIR}/muse_v2/MuseV2_SerialConnection.cpp	
   )
endif()

add_library(MitchV2 STATIC ${MITCH_V2_INCLUDES} ${MITCH_V2_SOURCES} ${Boost_LIBRARIES})
target_link_libraries(MitchV2 Connection)

install(TARGETS MitchV2
  # IMPORTANT: Add the Core library to the "export-set"
  EXPORT lib221eTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/mitch_v2"
    COMPONENT dev)

add_library(MuseV2 STATIC ${MUSE_V2_INCLUDES} ${MUSE_V2_SOURCES} ${Boost_LIBRARIES})
target_link_libraries(MuseV2 Connection)

install(TARGETS MuseV2
  # IMPORTANT: Add the Core library to the "export-set"
  EXPORT lib221eTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/muse_v2"
    COMPONENT dev)

## test
add_executable(MitchV2SerialTest "test/mitch_v2_serial_test.cpp")
target_link_libraries(MitchV2SerialTest MitchV2 Connection)

add_executable(MuseV2SerialTest "test/muse_v2_serial_test.cpp")
target_link_libraries(MuseV2SerialTest MuseV2 Connection)

if(UNIX)
    add_executable(MitchV2BLETest "test/mitch_v2_ble_test.cpp")
    target_link_libraries(MitchV2BLETest MitchV2 ${GATTLIB} pthread jsoncpp)

    # Add all targets to the build-tree export set
    export(TARGETS MitchV2 MuseV2 Connection MitchV2SerialTest MitchV2BLETest MuseV2SerialTest
    FILE "${PROJECT_BINARY_DIR}/lib221eTargets.cmake")
else()
    # Add all targets to the build-tree export set
    export(TARGETS MitchV2 MuseV2 Connection MitchV2SerialTest MuseV2SerialTest
    FILE "${PROJECT_BINARY_DIR}/lib221eTargets.cmake")
endif()

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE lib221e)

# Create the MitchConfig.cmake and MitchConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_INCLUDE_DIR}" "${PROJECT_BINARY_DIR}" "${INSTALL_LIB_DIR}")
configure_file(lib221eConfig.cmake.in
  "${PROJECT_BINARY_DIR}/lib221eConfig.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${LIB221E_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(lib221eConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/lib221eConfig.cmake" @ONLY)
# ... for both
configure_file(lib221eConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/lib221eConfigVersion.cmake" @ONLY)

# Install the MitchConfig.cmake and MitchConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/lib221eConfig.cmake"
  "${PROJECT_BINARY_DIR}/lib221eConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT lib221eTargets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)