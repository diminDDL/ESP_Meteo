cmake_minimum_required(VERSION 3.16.0)

if(NOT DEFINED EXTRA_COMPONENT_DIRS)
    set(EXTRA_COMPONENT_DIRS "")
endif()

list(APPEND EXTRA_COMPONENT_DIRS
    "${CMAKE_SOURCE_DIR}/src/bme280"
    "${CMAKE_SOURCE_DIR}/src/i2c_bus"
    "${CMAKE_SOURCE_DIR}/managed_components"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP_Meteo)
