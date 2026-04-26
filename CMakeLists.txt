cmake_minimum_required(VERSION 3.16)

# Use already-downloaded shared managed components from the parent project.
set(EXTRA_COMPONENT_DIRS
	"${CMAKE_SOURCE_DIR}/../managed_components"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sd_slideshow)
