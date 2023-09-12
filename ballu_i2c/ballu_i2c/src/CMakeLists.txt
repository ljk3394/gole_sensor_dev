cmake_minimum_required(VERSION 3.18)
set (CMAKE_CXX_STANDARD 14) # make_unique
project(
        ballu_cpp
        VERSION 0.1
        DESCRIPTION "I2C Devices for BALLU"
        LANGUAGES C CXX)

##################################################
set(PROJECT ballu_cpp)
set(COMPONENT devices)
set(LIBRARY i2c_devices)

##################################################
# refer to //marker_parser/python/CMakeLists.txt
set(CMAKE_SKIP_BUILD_RPATH FALSE)
#set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
#set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
#SET(CMAKE_INSTALL_RPATH "\${ORIGIN}")
SET(CMAKE_INSTALL_RPATH "\${ORIGIN}:\${ORIGIN}/../utils:\${INSTALL_RPATH}")

# https://serverfault.com/questions/279068/cant-find-so-in-the-same-directory-as-the-executable
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath='$ORIGIN'") # CMAKE_BUILD_RPATH_USE_ORIGIN. ?

list(APPEND CMAKE_INSTALL_RPATH "@executable_path" "@loader_path")

##################################################
#if(RPI)
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
#endif()
set(THREADS_PREFFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
if(RPI)
find_package(yaml-cpp REQUIRED)
endif()

##################################################
include_directories(
    ${CMAKE_SOURCE_DIR}/include/state_estimator
    ${CMAKE_SOURCE_DIR}/include/i2c_devices         # CMAKE_SOURCE_DIR: where the root CMakeListst.txt is
    ${CMAKE_SOURCE_DIR}/include/utils
    ${CMAKE_SOURCE_DIR}/include/shared_memory
    
    ${CMAKE_SOURCE_DIR}/lib/sh2/include
    ${CMAKE_SOURCE_DIR}/lib/argh/include
    ${CMAKE_SOURCE_DIR}/lib/json/include
)
# add_compile_options(-Wall)
link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils
    ${CMAKE_CURRENT_BINARY_DIR}/../utils
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared_memory
)
##################################################
add_library(${LIBRARY}
    STATIC # SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/i2c_device.cpp      # CMAKE_CURRENT_SOURCE_DIR: where this CMakeLists.txt is
    ${CMAKE_CURRENT_SOURCE_DIR}/encoder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imu.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/bno08x.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tof.cpp
)
target_link_libraries(${LIBRARY}
    INTERFACE
    rt
    pigpiod_if2
    sh2
)
target_link_libraries(${LIBRARY}
    PRIVATE
    Threads::Threads
)

##################################################
if(RPI)
    add_subdirectory(tools)
else()
    file(GLOB_RECURSE ARDUINO_SCRIPTS "tools/*.ino")
    install(FILES ${ARDUINO_SCRIPTS} DESTINATION ${PROJECT}/${COMPONENT})
endif()

install(TARGETS ${BINARY}
    LIBRARY
        DESTINATION ${PROJECT}/${COMPONENT}
)
###################################################
