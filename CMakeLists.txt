cmake_minimum_required(VERSION 3.16)

set(SDKCONFIG_DEFAULTS "sdkconfig.defaults")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
add_compile_options(-Wno-error=format)
project(ESPCast)
