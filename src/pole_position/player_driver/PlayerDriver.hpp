#pragma once

#include "util/logger/Logger.hpp"

#include "quartz/managers/input_manager/InputManager.hpp"
#include "quartz/scene/camera/Camera.hpp"
#include "quartz/scene/doodad/Doodad.hpp"

#include "pole_position/Loggers.hpp"

class PlayerDriver {
public: // classes and enums
    struct GearInformation {
        double accelerationRate_mps2;
        double maxSpeed_kph;
    };

public: // member functions
    PlayerDriver();

    void awakenCallback(quartz::scene::Doodad::AwakenCallbackParameters parameters);
    void fixedUpdateCallback(quartz::scene::Doodad::FixedUpdateCallbackParameters parameters);
    void updateCallback(quartz::scene::Doodad::UpdateCallbackParameters parameters);

    USE_LOGGER(PLAYER);

private: // member functions
    void actuationFixedUpdate(
        const quartz::managers::InputManager& inputManager,
        quartz::scene::Doodad* const p_doodad,
        const double tickTimeDelta,
        const double currentSpeed_mps
    );
    void forwardAcceleration(
        quartz::scene::Doodad* const p_doodad,
        const double tickTimeDelta,
        const double actuationAmount,
        const double currentSpeed_mps
    );
    void forwardBrake(
        quartz::scene::Doodad* const p_doodad,
        const double tickTimeDelta,
        const double actuationAmount,
        const double currentSpeed_mps
    );
    void reverseAcceleration(
        quartz::scene::Doodad* const p_doodad,
        const double tickTimeDelta,
        const double actuationAmount,
        const double currentSpeed_mps
    );
    void reverseBrake(
        quartz::scene::Doodad* const p_doodad,
        const double tickTimeDelta,
        const double actuationAmount,
        const double currentSpeed_mps
    );

    void cameraUpdate(
        const math::Vec3& currentCarVelocity,
        const math::Vec3& currentCarPosition,
        const math::Quaternion& currentCarRotation
    );

private: // member variables
    std::vector<GearInformation> m_gearInformations;
    uint32_t m_currentGear;

    math::Vec3 m_currentDesiredDirection;
    math::Vec3 m_currentVelocity_mps;

    quartz::scene::Camera m_camera;
    double m_cameraLookAheadAmount;
    double m_cameraHorizontalOffsetAmount;
    double m_cameraVerticalOffsetAmount;
};

