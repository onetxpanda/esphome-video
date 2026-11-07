# Decision logic of the esphome-h264 build guard, kept separate from the IDF
# plumbing so it can be tested with `cmake -P` (tests/cmake/test_guard.cmake).
#
# rev:  CONFIG_ESP32P4_REV_MIN_FULL (e.g. 301)
# dirs: esp_h264's configured include directories
# Stops the build unless exactly the matching register set is configured.
function(esphome_h264_verify_register_set rev dirs)
    if(NOT rev)
        message(FATAL_ERROR "esphome-h264: CONFIG_ESP32P4_REV_MIN_FULL is not set, "
            "so the H.264 register layout cannot be checked.")
    endif()
    if(rev GREATER_EQUAL 300)
        set(want "hw_ver3")
        set(wrong "hw_ver1")
    else()
        set(want "hw_ver1")
        set(wrong "hw_ver3")
    endif()
    string(FIND "${dirs}" "/hw/soc/esp32p4/${want}" has_want)
    string(FIND "${dirs}" "/hw/soc/esp32p4/${wrong}" has_wrong)
    if(has_want EQUAL -1 OR NOT has_wrong EQUAL -1)
        message(FATAL_ERROR "esphome-h264: esp_h264 is configured with the wrong H.264 "
            "register definitions for this chip (revision ${rev} needs ${want}). "
            "The firmware would misprogram the encoder, so the build is stopped. "
            "esp_h264 include directories: ${dirs}")
    endif()
    message(STATUS "esphome-h264: esp_h264 uses ${want} register definitions "
        "for chip revision ${rev} (checked)")
endfunction()
