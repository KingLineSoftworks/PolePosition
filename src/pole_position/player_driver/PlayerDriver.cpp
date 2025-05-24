#include "math/transform/Vec3.hpp"

#include "quartz/managers/input_manager/InputManager.hpp"
#include "quartz/physics/rigid_body/RigidBody.hpp"
#include "quartz/scene/doodad/Doodad.hpp"
#include "quartz/scene/scene/Scene.hpp"
#include "util/logger/Logger.hpp"

#include "pole_position/player_driver/PlayerDriver.hpp"

PlayerDriver::PlayerDriver() :
    m_gearInformations({{10, 15}, {1.75, 27.5}, {1.25, 35}}),
    m_currentGear(0),
    m_currentDesiredDirection(),
    m_currentVelocity_mps(),
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
    quartz::scene::Doodad::FixedUpdateCallbackParameters parameters
) {
    // Compute the current velocity
    m_currentVelocity_mps = parameters.p_doodad->getRigidBodyOptionalReference()->getLinearVelocity_mps();

    // Compute the current speed from the current velocity
    const double currentSpeed_mps = m_currentVelocity_mps.magnitude();

    // Steer the car
    
    // Determine the desired direction of travel accounting for how the car has steered this frame
    m_currentDesiredDirection = math::Vec3::Forward;
    
    // Accelerate / brake
    this->actuationFixedUpdate(parameters.inputManager, parameters.p_doodad, parameters.ticksPerSecond, currentSpeed_mps);
}

void
PlayerDriver::updateCallback(
    quartz::scene::Doodad::UpdateCallbackParameters parameters
) {
    this->cameraUpdate(
        m_currentVelocity_mps,
        parameters.p_doodad->getTransform().position,
        parameters.p_doodad->getTransform().rotation
    );
}

void
PlayerDriver::actuationFixedUpdate(
    const quartz::managers::InputManager& inputManager,
    quartz::scene::Doodad* const p_doodad,
    const double ticksPerSecond,
    const double currentSpeed_mps
) {
    // get input (-1 to 1) from S and W
    double actuationAmount = 0;
    if (inputManager.getKeyDown_w()) {
        actuationAmount += 1.0;
    }
    if (inputManager.getKeyDown_s()) {
        actuationAmount -= 1.0;
    }

    // We are prepared to go forwards
    if (currentSpeed_mps >= 0.0 && actuationAmount > 0.0) {
        // Accelerate +
        return this->forwardAcceleration(p_doodad, ticksPerSecond, actuationAmount, currentSpeed_mps);
    }
    if (currentSpeed_mps > 0.0 && actuationAmount < 0.0) {
        // Brake -
        return this->forwardBrake(p_doodad, ticksPerSecond, actuationAmount, currentSpeed_mps);
    }

    // We are prepared to go in reverse
    if (currentSpeed_mps <= 0.0 && actuationAmount < 0.0) {
        // Accelerate +
        return this->reverseAcceleration(p_doodad, ticksPerSecond, actuationAmount, currentSpeed_mps);
    }
    if (currentSpeed_mps < 0.0 && actuationAmount > 0.0) {
        // Brake +
        return this->reverseBrake(p_doodad, ticksPerSecond, actuationAmount, currentSpeed_mps);
    }

    /** @todo 2025/03/02 Slow down gradually (air brake) */
}

void
PlayerDriver::forwardAcceleration(
    quartz::scene::Doodad* const p_doodad,
    const double ticksPerSecond,
    const double actuationAmount,
    const double currentSpeed_mps
) {
    LOG_FUNCTION_SCOPE_TRACEthis("");

    const GearInformation& gearInformation = m_gearInformations[m_currentGear];
    const double accelerationRate_mps2 = gearInformation.accelerationRate_mps2;
    const double maxSpeed_kph = gearInformation.maxSpeed_kph;

    LOG_TRACEthis("Acceleration rate (mps2): {}", accelerationRate_mps2);
    LOG_TRACEthis("Max speed         (kph) : {}", maxSpeed_kph);

    // kilometers per hour -> meters per hour -> meters per second
    const double maxSpeed_mps = (maxSpeed_kph * 1000.0) / (60.0 * 60.0);
    LOG_TRACEthis("Max speed         (mps) : {}", maxSpeed_mps);

    LOG_TRACEthis("Current speed     (mps) : {}", currentSpeed_mps);
    if (currentSpeed_mps >= maxSpeed_mps) {
        LOG_TRACEthis("Current speed exceeds max speed, not accelerating further");
        return;
    }

    // - based on the acceleration rate in mps2, we can determine the delta in velocity in mps at 1 second from now
    // - we can convert the velocity delta in mps at 1 second from now to velocity delta in mps at 1 tick from now
    // - we can convert the velocity delta in mps to mpt
    // 
    // - we can convert the current velocity in mps to mpt
    // - from the current velocity in mpt and the velocity delta in mpt, we can determine the velocity in mpt at 1 tick from now
    // - we can convert the velocity in mpt at 1 tick from now to velocity in mps at 1 tick from now
    // - we can set the rigidbody's velocity to be the velocity in mps at 1 tick from now

    // -----

    // const math::Vec3 velocityDelta_mps_1sFromNow = m_currentDesiredDirection.normalize() * (accelerationRate_mps2 * actuationAmount);
    // const math::Vec3 velocityDelta_mpt_1sFromNow = velocityDelta_mps_1sFromNow / ticksPerSecond;

    // const math::Vec3 currentLinearVelocity_mps = rigidBody.getLinearVelocity_mps();
    // const math::Vec3 currentLinearVelocity_mpt = currentLinearVelocity_mps / ticksPerSecond;
    // const math::Vec3 desiredLinearVelocity_mpt_1sFromNow = currentLinearVelocity_mpt + velocityDelta_mpt_1sFromNow;
    // const math::Vec3 desiredLinearVelocity_mps_1sFromNow = desiredLinearVelocity_mpt_1sFromNow * ticksPerSecond;
    
    // rigidBody.setLinearVelocity_mps(desiredLinearVelocity_mps_1sFromNow);

    // -----

    // - based on the acceleration rate in mps2, we can determine the delta in velocity in mps at 1 second from now
    // - we can convert the velocity delta in mps at 1 second from now to velocity delta in mps at 1 tick from now
    //
    // - we can calculate the velocity in mps at 1 tick from now from current velocity in mps and velocity delta in mps at 1 tick from now
    // - we can update the rigidbody's velocity in mps with the newly calculated velocity in mps

    LOG_TRACEthis("");
    LOG_TRACEthis("Direction: {}", m_currentDesiredDirection.toString());
    LOG_TRACEthis("Actuated acceleration (mps2): {}", (accelerationRate_mps2 * actuationAmount));

    const math::Vec3 velocityDelta_mps_1sFromNow = m_currentDesiredDirection.normalize() * (accelerationRate_mps2 * actuationAmount);
    const math::Vec3 velocityDelta_mps_1tFromNow = velocityDelta_mps_1sFromNow / ticksPerSecond;

    LOG_TRACEthis("Velocity delta in 1 second from now (mps): {}", velocityDelta_mps_1sFromNow.toString());
    LOG_TRACEthis("Velocity delta in 1 tick   from now (mps): {}", velocityDelta_mps_1tFromNow.toString());

    const math::Vec3 velocity_mps_1tFromNow = m_currentVelocity_mps + velocityDelta_mps_1tFromNow;

    LOG_TRACEthis("Current velocity            (mps): {}", m_currentVelocity_mps.toString());
    LOG_TRACEthis("Velocity in 1 tick from now (mps): {}", velocityDelta_mps_1sFromNow.toString());

    quartz::physics::RigidBody& rigidBody = p_doodad->getRigidBodyOptionalReference().value();
    rigidBody.setLinearVelocity_mps(velocity_mps_1tFromNow);
}

void
PlayerDriver::forwardBrake(
    UNUSED quartz::scene::Doodad* const p_doodad,
    UNUSED const double ticksPerSecond,
    UNUSED const double actuationAmount,
    UNUSED const double currentSpeed_mps
) {

}

void
PlayerDriver::reverseAcceleration(
    UNUSED quartz::scene::Doodad* const p_doodad,
    UNUSED const double ticksPerSecond,
    UNUSED const double actuationAmount,
    UNUSED const double currentSpeed_mps
) {

}

void
PlayerDriver::reverseBrake(
    UNUSED quartz::scene::Doodad* const p_doodad,
    UNUSED const double ticksPerSecond,
    UNUSED const double actuationAmount,
    UNUSED const double currentSpeed_mps
) {

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

