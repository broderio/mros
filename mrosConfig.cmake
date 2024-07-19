# Get the directory containing this file.
get_filename_component(MROS_CURRENT_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

# Get a list of all library files in the /usr/local/lib/mros directory.
file(GLOB MROS_LIBRARIES "/usr/local/lib/mros/*.a")

# Set the location of your headers.
set(MROS_INCLUDE_DIRS "/usr/local/include/mros")

# Load the settings into the current CMake project.
include(${MROS_CURRENT_DIR}/mrosTargets.cmake)