# ESP-DL benchmark for the quantized detectors (scripts/esp/int8_bench.py).
# A separate app: flashing it replaces the AGV firmware until that is
# flashed again.
cmake_minimum_required(VERSION 3.22)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_dl_bench)
