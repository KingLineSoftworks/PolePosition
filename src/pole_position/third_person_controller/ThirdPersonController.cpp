#include "math/transform/Vec3.hpp"

#include "util/logger/Logger.hpp"

#include "quartz/scene/doodad/Doodad.hpp"
#include "quartz/scene/scene/Scene.hpp"

#include "pole_position/third_person_controller/ThirdPersonController.hpp"

ThirdPersonController::ThirdPersonController() :
    m_camera(
        75.0f,
        { 1.25f, 3.0f, 10.0f },
        math::Vec3::Backward
    ),
    m_cameraHorizontalOffset(5.0),
    m_cameraVerticalOffset(3.0),
    m_maxHorizontalMovementSpeed(7.0),
    m_currentHorizontalMovementSpeed(0.0)
{}

void
ThirdPersonController::awakenCallback(
    quartz::scene::Doodad::AwakenCallbackParameters parameters
) {
    LOG_FUNCTION_SCOPE_INFOthis("");
    parameters.p_scene->setCamera(m_camera);
}

void
ThirdPersonController::fixedUpdateCallback(
    quartz::scene::Doodad::FixedUpdateCallbackParameters parameters
) {
    this->movementFixedUpdate(parameters.inputManager, parameters.p_doodad, parameters.ticksPerSecond);
}

void
ThirdPersonController::updateCallback(
    quartz::scene::Doodad::UpdateCallbackParameters parameters
) {
    this->cameraUpdate(parameters.inputManager, parameters.p_doodad);
}

void
ThirdPersonController::movementFixedUpdate(
    const quartz::managers::InputManager& inputManager,
    quartz::scene::Doodad* const p_doodad,
    UNUSED const double ticksPerSecond
) {
    // Get the direction we are facing
    // @todo Get this from the camera dotted with the horizon, normalized
    const math::Vec3 forwardDirection(1, 0, 0); 
    const math::Vec3 rightDirection(0, 0, 1);

    math::Vec3 horizontalMovementDirection(0, 0, 0);

    // Front and back
    if (inputManager.getKeyDown_w()) {
        horizontalMovementDirection += forwardDirection;
    }
    if (inputManager.getKeyDown_s()) {
        horizontalMovementDirection -= forwardDirection;
    }

    // Left and right
    if (inputManager.getKeyDown_a()) {
        horizontalMovementDirection -= rightDirection;
    }
    if (inputManager.getKeyDown_d()) {
        horizontalMovementDirection += rightDirection;
    }

    // Convert horizontal movement direction to velocity
    horizontalMovementDirection.normalize();
    const math::Vec3 currentHorizontalMovementVelocity = horizontalMovementDirection * m_maxHorizontalMovementSpeed;

    quartz::physics::RigidBody& rigidBody = p_doodad->getRigidBodyOptionalReference().value();
    rigidBody.setLinearVelocity_mps(currentHorizontalMovementVelocity);

    m_currentHorizontalMovementSpeed = currentHorizontalMovementVelocity.magnitude();
}
 
void
ThirdPersonController::cameraUpdate(
    UNUSED const quartz::managers::InputManager& inputManager,
    quartz::scene::Doodad* const p_doodad
) {
    // Get doodad information
    const math::Vec3 doodadPosition = p_doodad->getTransform().position;
    const math::Vec3 doodadForwardDirection(1, 0, 0);
    const math::Vec3 doodadUpDirection(0, 1, 0);

    // Rotate camera around player according to mouse input
    
    // Determine position of camera
    const math::Vec3 cameraPosition =
        doodadPosition -
        (doodadForwardDirection * m_cameraHorizontalOffset) +
        (doodadUpDirection * m_cameraVerticalOffset);
    m_camera.setPosition(cameraPosition);
    
    // Point the camera at the doodad
    m_camera.lookAtPosition(doodadPosition);
}

