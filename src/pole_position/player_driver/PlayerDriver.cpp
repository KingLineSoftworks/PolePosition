#include "math/transform/Vec3.hpp"

#include "quartz/managers/input_manager/InputManager.hpp"
#include "quartz/scene/doodad/Doodad.hpp"
#include "quartz/scene/scene/Scene.hpp"
#include "util/logger/Logger.hpp"
#include "pole_position/player_driver/PlayerDriver.hpp"

PlayerDriver::PlayerDriver() :
    m_currentDesiredDirection(),
    m_currentVelocity(),
    m_camera(
        75.0f,
        { 1.25f, 3.0f, 10.0f },
        math::Vec3::Backward
    ),
    m_cameraLookAheadAmount(5.0),
    m_cameraHorizontalOffsetAmount(7.5),
    m_cameraVerticalOffsetAmount(3.5)
{}

void
PlayerDriver::awakenCallback(
    quartz::scene::Doodad::AwakenCallbackParameters parameters
) {
    LOG_FUNCTION_SCOPE_INFOthis("");
    parameters.p_scene->setCamera(m_camera);
}

void
PlayerDriver::fixedUpdateCallback(
    UNUSED quartz::scene::Doodad::FixedUpdateCallbackParameters parameters
) {
    // Turn the car
    
    // Determine the desired direction of travel
    
    // Accelerate / brake
}

void
PlayerDriver::updateCallback(
    quartz::scene::Doodad::UpdateCallbackParameters parameters
) {
    this->cameraUpdate(
        m_currentVelocity,
        parameters.p_doodad->getTransform().position,
        parameters.p_doodad->getTransform().rotation
    );
}

void
PlayerDriver::cameraUpdate(
    UNUSED const math::Vec3& currentCarVelocity,
    const math::Vec3& currentCarPosition,
    const math::Quaternion& currentCarRotation
) {
    math::Vec3 directionVector = currentCarRotation.getDirectionVector(); // Get the direction the car is facing
    directionVector.y = 0; // to project onto horizontal plane
    directionVector.normalize();
   
    const math::Vec3 lookAheadOffset = m_cameraLookAheadAmount * directionVector;
    const math::Vec3 horizontalOffset = m_cameraHorizontalOffsetAmount * (-1.0 * directionVector);
    const math::Vec3 verticalOffset = m_cameraVerticalOffsetAmount * math::Vec3::Up;

    m_camera.setPosition(currentCarPosition + horizontalOffset + verticalOffset);

    // Point the camera at the car
    m_camera.lookAtPosition(currentCarPosition + lookAheadOffset);
}

