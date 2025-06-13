#include <algorithm>

#include "util/logger/Logger.hpp"

#include "math/transform/Vec3.hpp"

#include "quartz/scene/camera/Camera.hpp"

#include "quartz/scene/doodad/Doodad.hpp"
#include "quartz/scene/scene/Scene.hpp"
#include "util/macros.hpp"

#include "pole_position/third_person_controller/ThirdPersonController.hpp"

math::Vec3
ThirdPersonController::calculateCameraOffset(
    const math::Vec3& cameraLookDirection,
    const double horizontalOffset,
    const double verticalOffset
) {
    QUARTZ_ASSERT(cameraLookDirection.isNormalized(), "Camera look direction must be normalized");

    const math::Vec3 cameraRightDirection = cameraLookDirection.cross(math::Vec3::Up).normalize();
    QUARTZ_ASSERT(cameraRightDirection.isNormalized(), "Camera right direction must be normalized");

    const math::Vec3 cameraUpDirection = cameraRightDirection.cross(cameraLookDirection).normalize();
    QUARTZ_ASSERT(cameraUpDirection.isNormalized(), "Camera up direction must be normalized");

    const math::Vec3 cameraHorizontalOffset = horizontalOffset * cameraRightDirection;
    const math::Vec3 cameraVerticalOffset = verticalOffset * cameraUpDirection;

    return cameraHorizontalOffset + cameraVerticalOffset;
}

ThirdPersonController::ThirdPersonController() :
    m_camera(
        75.0f,
        { 1.25f, 3.0f, 10.0f },
        math::Vec3::Backward
    ),
    m_cameraSensitivity(2.0),
    m_cameraDistanceSensitivity(0.25),
    m_cameraFocalPointHorizontalOffset(2.0),
    m_cameraFocalPointVerticalOffset(2.0),
    m_cameraDistanceMin(1.0),
    m_cameraDistanceMax(200.0),
    m_cameraDistanceCurrent(10.0),
    m_maxHorizontalMovementSpeed(15.0),
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
    const math::Vec3 forwardDirection = m_camera.getLookDirection().getProjectionOntoPlane(math::Vec3::Up).normalize();

    const math::Quaternion doodadRotation = math::Quaternion::fromEulerAngles(-1 * m_camera.getEulerAngles().yawDegrees, 0, 0);
    p_doodad->setRotation(doodadRotation);

    const math::Vec3 rightDirection = forwardDirection.cross(math::Vec3::Up).normalize();

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
    const quartz::managers::InputManager& inputManager,
    quartz::scene::Doodad* const p_doodad
) {
    // Rotate camera according to mouse input
    const quartz::scene::Camera::EulerAngles previousEulerAngles = m_camera.getEulerAngles();
    const double calibratedMousePositionOffset_x = inputManager.getMousePositionOffset_x() * m_cameraSensitivity;
    const double calibratedMousePositionOffset_y = inputManager.getMousePositionOffset_y() * m_cameraSensitivity;
    const double updatedPitch = std::clamp(previousEulerAngles.pitchDegrees + calibratedMousePositionOffset_y, -89.5, 89.5);
    const double updatedYaw = glm::mod(previousEulerAngles.yawDegrees - calibratedMousePositionOffset_x, 360.0);
    m_camera.setEulerAngles({updatedYaw, updatedPitch, previousEulerAngles.rollDegrees});

    // Update the camera distance according to mouse input
    const double calibratedMouseDistanceOffset = inputManager.getScrollOffset_y() * m_cameraDistanceSensitivity * -1.0;
    m_cameraDistanceCurrent += calibratedMouseDistanceOffset;
    m_cameraDistanceCurrent = std::clamp(m_cameraDistanceCurrent, m_cameraDistanceMin, m_cameraDistanceMax);

    // Move camera based on doodad's position and camera's direction
    const math::Vec3 cameraInitialPosition = p_doodad->getTransform().position - m_camera.getLookDirection() * m_cameraDistanceCurrent;
    const math::Vec3 cameraPositionOffset = ThirdPersonController::calculateCameraOffset(
        m_camera.getLookDirection(),
        m_cameraFocalPointHorizontalOffset,
        m_cameraFocalPointVerticalOffset
    );
    const math::Vec3 cameraPosition = cameraInitialPosition + cameraPositionOffset;
    m_camera.setPosition(cameraPosition);
}

