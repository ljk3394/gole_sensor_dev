
##################################################
set(BIN_CALIB_ENC calibrate_encoder)
add_executable(${BIN_CALIB_ENC} ${CMAKE_CURRENT_SOURCE}/calibrate_encoder.cpp)
target_link_libraries(${BIN_CALIB_ENC} i2c_devices pigpiod_if2 yaml-cpp)

##################################################
set(BIN_ECHO_ENC echo_encoder)
add_executable(${BIN_ECHO_ENC} ${CMAKE_CURRENT_SOURCE}/echo_encoder.cpp)
target_link_libraries(${BIN_ECHO_ENC} i2c_devices Threads::Threads)

##################################################
set(BIN_ECHO_IMU echo_imu)
add_executable(${BIN_ECHO_IMU} ${CMAKE_CURRENT_SOURCE}/echo_imu.cpp)
target_link_libraries(${BIN_ECHO_IMU} i2c_devices Threads::Threads )

##################################################
set(BIN_ECHO_TOF echo_tof)
add_executable(${BIN_ECHO_TOF} ${CMAKE_CURRENT_SOURCE}/echo_tof.cpp)
target_link_libraries(${BIN_ECHO_TOF} i2c_devices Threads::Threads utils yaml-cpp )

##################################################
set(BIN_ECHO_SENSORS echo_sensors)
add_executable(${BIN_ECHO_SENSORS} ${CMAKE_CURRENT_SOURCE}/echo_sensors.cpp)
target_link_libraries(${BIN_ECHO_SENSORS} i2c_devices utils Threads::Threads yaml-cpp shared_memory)

###################################################
set(BIN_TEST_CHAINS test_chains)
add_executable(${BIN_TEST_CHAINS} ${CMAKE_CURRENT_SOURCE}/test.cpp)
target_link_libraries(${BIN_TEST_CHAINS} rt pigpiod_if2 i2c_devices Threads::Threads)

###################################################
set(BIN_UPDATE_ENCODER_ID update_encoder_id)
add_executable(${BIN_UPDATE_ENCODER_ID} ${CMAKE_CURRENT_SOURCE}/update_encoder_id.cpp)
target_link_libraries(${BIN_UPDATE_ENCODER_ID} pigpiod_if2 i2c_devices)

###################################################
if(RPI)
    install(TARGETS ${BIN_CALIB_ENC} ${BIN_ECHO_ENC} ${BIN_ECHO_IMU} ${BIN_ECHO_SENSORS} ${BIN_TEST_CHAINS} ${BIN_UPDATE_ENCODER_ID} ${BIN_ECHO_TOF}
        RUNTIME
        DESTINATION ${PROJECT}/${COMPONENT}
    )
    file(GLOB_RECURSE YAML_FILES "${CMAKE_CURRENT_SOURCE}*.yaml")
    install(FILES ${YAML_FILES} DESTINATION ${PROJECT}/${COMPONENT})
endif()

