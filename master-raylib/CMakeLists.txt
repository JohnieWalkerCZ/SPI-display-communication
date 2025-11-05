cmake_minimum_required(VERSION 3.5)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(master)

# Add raylib component
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/components)
list(APPEND EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/../components)

# Add raylib as a required component
set(COMPONENTS main raylib)
