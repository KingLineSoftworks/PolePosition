#pragma once

#include "util/logger/Logger.hpp"

#include "quartz/managers/input_manager/InputManager.hpp"
#include "quartz/scene/camera/Camera.hpp"
#include "quartz/scene/doodad/Doodad.hpp"

#include "pole_position/Loggers.hpp"

class ThirdPersonController {
public: // classes and enums

public: // member functions
    ThirdPersonController();

    void awakenCallback(quartz::scene::Doodad::AwakenCallbackParameters parameters);
    void fixedUpdateCallback(quartz::scene::Doodad::FixedUpdateCallbackParameters parameters);
    void updateCallback(quartz::scene::Doodad::UpdateCallbackParameters parameters);

    USE_LOGGER(PLAYER);

private: // helpers
    static math::Vec3 calculateCameraOffset(
        const math::Vec3& cameraLookDirection,
        const double horizontalOffset,
        const double verticalOffset
    );

private: // member functions
    void movementFixedUpdate(
        const quartz::managers::InputManager& inputManager,
        quartz::scene::Doodad* const p_doodad,
        const double ticksPerSecond
    );

    void cameraUpdate(
        const quartz::managers::InputManager& inputManager,
        quartz::scene::Doodad* const p_doodad
    );

private: // member variables
    quartz::scene::Camera m_camera;
    double m_cameraSensitivity;
    double m_cameraFocalPointHorizontalOffset;
    double m_cameraFocalPointVerticalOffset;
    double m_cameraDistance;

    double m_maxHorizontalMovementSpeed;
    double m_currentHorizontalMovementSpeed;
};

