cmake_minimum_required(VERSION 3.14)
project(pypilot-sensors VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(PYPILOT_DATA_MODEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pypilot-data-model/src" CACHE PATH "Path to pypilot-data-model src")
set(PYPILOT_ALGORITHMS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pypilot-algorithms/src" CACHE PATH "Path to pypilot-algorithms src")
set(PYPILOT_NMEA0183_CONNECTOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pypilot-nmea0183-connector/src" CACHE PATH "Path to pypilot-nmea0183-connector src")
set(PYPILOT_SIGNALK_CONNECTOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pypilot-signalk-connector/src" CACHE PATH "Path to pypilot-signalk-connector src")
set(PYPILOT_SERVO_PROTOCOL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pypilot-servo-protocol/src" CACHE PATH "Path to pypilot-servo-protocol src")

add_library(pypilot_sensors STATIC src/pypilot_sensors.cpp)
target_include_directories(pypilot_sensors PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${PYPILOT_DATA_MODEL_DIR}
    ${PYPILOT_ALGORITHMS_DIR}
    ${PYPILOT_NMEA0183_CONNECTOR_DIR}
    ${PYPILOT_SIGNALK_CONNECTOR_DIR}
    ${PYPILOT_SERVO_PROTOCOL_DIR})

option(PYPILOT_SENSORS_BUILD_TESTS "Build pypilot-sensors tests" ON)
if(PYPILOT_SENSORS_BUILD_TESTS)
    enable_testing()
    function(add_sensor_test name source)
        add_executable(${name} ${source})
        target_link_libraries(${name} PRIVATE pypilot_sensors)
        add_test(NAME ${name} COMMAND ${name})
    endfunction()

    add_sensor_test(test_data_model_writer tests/test_data_model_writer.cpp)
    add_sensor_test(test_nmea_adapter tests/test_nmea_adapter.cpp)
    add_sensor_test(test_signalk_adapter tests/test_signalk_adapter.cpp)
    add_sensor_test(test_servo_rudder_adapter tests/test_servo_rudder_adapter.cpp)
    add_sensor_test(test_replay_adapter tests/test_replay_adapter.cpp)
endif()
