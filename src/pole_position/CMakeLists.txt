#====================================================================
# The Pole Position executable
#====================================================================
set(POLE_POSITION_APPLICATION_NAME "PolePosition")

message(STATUS "Adding application with name ${POLE_POSITION_APPLICATION_NAME}")
add_executable(
    ${POLE_POSITION_APPLICATION_NAME}
    main.cpp
    Boilerplate.hpp
    Boilerplate.cpp
    scene/SceneParameters.hpp
    scene/SceneParameters.cpp
    third_person_controller/ThirdPersonController.hpp
    third_person_controller/ThirdPersonController.cpp
)

target_include_directories(
    ${POLE_POSITION_APPLICATION_NAME}
    PRIVATE
    ${QUARTZ_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/.."
)

target_compile_options(
    ${POLE_POSITION_APPLICATION_NAME}
    PUBLIC
    ${QUARTZ_CMAKE_CXX_FLAGS}
)

# pre compile definitions for the target
target_compile_definitions(
    ${POLE_POSITION_APPLICATION_NAME}
    PUBLIC ${QUARTZ_COMPILE_DEFINITIONS}
    PUBLIC APPLICATION_NAME="${POLE_POSITION_APPLICATION_NAME}"
    PUBLIC APPLICATION_VERSION
    PUBLIC APPLICATION_MINOR_VERSION=${APPLICATION_MINOR_VERSION}
    PUBLIC APPLICATION_PATCH_VERSION=${APPLICATION_PATCH_VERSION}
    PUBLIC APPLICATION_MAJOR_VERSION=${APPLICATION_MAJOR_VERSION}
)

# Link everything to the application
target_link_libraries(
    ${POLE_POSITION_APPLICATION_NAME}

    # Vendor (leaving this commented section in here for ease of placement in the future when prototyping)
    PRIVATE
    reactphysics3d

    # Math
    PRIVATE
    MATH_Transform

    # Utility
    PRIVATE
    UTIL_Logger

    # Quartz
    PRIVATE
    QUARTZ_Application
)

