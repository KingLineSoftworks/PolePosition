# ====================================================================
# Pole Position project and version
# ====================================================================

set(PROJECT_NAME "Pole Position")

cmake_minimum_required(VERSION 3.20)

cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0069 NEW)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/Quartz/cmake;${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(QuartzVersion)
include(Version)

project(${PROJECT_NAME} VERSION ${APPLICATION_MAJOR_VERSION}.${APPLICATION_MINOR_VERSION}.${APPLICATION_PATCH_VERSION} LANGUAGES CXX C)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_SHADER_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/spv)
file(MAKE_DIRECTORY ${CMAKE_SHADER_OUTPUT_DIRECTORY})

if (EXISTS "${CMAKE_BINARY_DIR}/compile_commands.json")
    message(STATUS "Creating compile_commands.json link in project root directory")
    file(CREATE_LINK "${CMAKE_BINARY_DIR}/compile_commands.json" "${PROJECT_SOURCE_DIR}/compile_commands.json")
else ()
    message(STATUS "CMake binary directory does not contain compile_commands.json file. This is probably the first time running cmake")
endif ()

# ====================================================================
# Pole Position project and version
# ====================================================================

# Quartz
set(QUARTZ_COMPILE_DEMO_APPLICATION OFF CACHE BOOL "" FORCE)
add_subdirectory("${PROJECT_SOURCE_DIR}/Quartz")

# Pole Position
add_subdirectory("${PROJECT_SOURCE_DIR}/src/pole_position")

