# esphome-h264 build guard.
#
# esp_h264 picks its register definitions (hw/soc/esp32p4/hw_ver1 or hw_ver3)
# from the chip revision at configure time. Building against the wrong set
# produces firmware that misprograms the encoder (on rev 3 silicon the v3
# registers, such as the DMA bytes-per-pixel register needed for RGB input,
# only exist in hw_ver3). esp_h264's own revision lookup prints "not defined"
# in some configure passes, so don't trust it: once every component is
# configured, check what esp_h264 was actually given, and stop the build if
# it does not match the configured chip revision.

if(CMAKE_SCRIPT_MODE_FILE)
    # IDF's dependency-scan passes run in script mode; nothing to check there.
    return()
endif()

include(${CMAKE_CURRENT_LIST_DIR}/esphome_h264_guard.cmake)

function(_esphome_h264_check_build rev)
    idf_build_get_property(target IDF_TARGET)
    if(NOT target STREQUAL "esp32p4")
        return()
    endif()
    idf_build_get_property(components BUILD_COMPONENTS)
    if(NOT "espressif__esp_h264" IN_LIST components)
        message(FATAL_ERROR "esphome-h264: esp_h264 is not part of the build.")
    endif()
    idf_component_get_property(lib espressif__esp_h264 COMPONENT_LIB)
    get_target_property(dirs ${lib} INCLUDE_DIRECTORIES)
    esphome_h264_verify_register_set("${rev}" "${dirs}")
endfunction()

# The chip configuration is loaded when project includes are processed;
# capture the revision now and check once every component is configured.
cmake_language(DEFER DIRECTORY ${CMAKE_SOURCE_DIR}
    CALL _esphome_h264_check_build "${CONFIG_ESP32P4_REV_MIN_FULL}")
