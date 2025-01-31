#pragma once

#include "util/logger/Logger.hpp"

#include "quartz/managers/input_manager/InputManager.hpp"
#include "quartz/scene/camera/Camera.hpp"
#include "quartz/scene/doodad/Doodad.hpp"

#include "pole_position/Loggers.hpp"

class PlayerDriver {
public: // member functions
    PlayerDriver();

    void awakenCallback(quartz::scene::Doodad::AwakenCallbackParameters parameters);
    void fixedUpdateCallback(quartz::scene::Doodad::FixedUpdateCallbackParameters parameters);
    void updateCallback(quartz::scene::Doodad::UpdateCallbackParameters parameters);

    USE_LOGGER(PLAYER);

private: // member functions
    
    void cameraUpdate(
        const math::Vec3& currentCarVelocity,
        const math::Vec3& currentCarPosition,
        const math::Quaternion& currentCarRotation
    );

private: // member variables
    math::Vec3 m_currentDesiredDirection;
    math::Vec3 m_currentVelocity;

    quartz::scene::Camera m_camera;
    double m_cameraLookAheadAmount;
    double m_cameraHorizontalOffsetAmount;
    double m_cameraVerticalOffsetAmount;
};

