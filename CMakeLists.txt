# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
# idf_build_set_property(MINIMAL_BUILD ON)  # Disabled to include WiFi and storage
set(EXTRA_COMPONENT_DIRS )

# Use custom partition table with SPIFFS storage partition
set(PARTITION_CSV_PATH ${CMAKE_CURRENT_SOURCE_DIR}/partitions.csv)

project(ESP)
