# Runs one case of the build guard's decision. Invoked by run_tests.py as:
#   cmake -DREV=<rev> -DDIRS=<dirs> -P test_guard.cmake
# Exit status 0 = guard accepted, non-zero = guard stopped the build.
include(${CMAKE_CURRENT_LIST_DIR}/../../components/h264_encoder/esphome_h264_guard/esphome_h264_guard.cmake)
esphome_h264_verify_register_set("${REV}" "${DIRS}")
