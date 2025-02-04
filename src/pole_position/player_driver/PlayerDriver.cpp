#include "math/transform/Vec3.hpp"

#include "quartz/managers/input_manager/InputManager.hpp"
#include "quartz/physics/rigid_body/RigidBody.hpp"
#include "quartz/scene/doodad/Doodad.hpp"
#include "quartz/scene/scene/Scene.hpp"
#include "util/logger/Logger.hpp"

#include "pole_position/player_driver/PlayerDriver.hpp"

PlayerDriver::PlayerDriver() :
    m_gearInformations({{0.7, 10}, {0.4, 25}, {0.2, 35}}),
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

    // Turn the car
    
    // Determine the desired direction of travel
    m_currentDesiredDirection = math::Vec3::Forward;
    
    // Accelerate / brake
    this->actuationFixedUpdate(parameters.inputManager, parameters.p_doodad, parameters.tickTimeDelta, currentSpeed_mps);
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
    const double tickTimeDelta,
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
        return this->forwardAcceleration(p_doodad, tickTimeDelta, actuationAmount, currentSpeed_mps);
    }
    if (currentSpeed_mps > 0.0 && actuationAmount < 0.0) {
        // Brake -
        return this->forwardBrake(p_doodad, tickTimeDelta, actuationAmount, currentSpeed_mps);
    }

    // We are prepared to go in reverse
    if (currentSpeed_mps <= 0.0 && actuationAmount < 0.0) {
        // Accelerate +
        return this->reverseAcceleration(p_doodad, tickTimeDelta, actuationAmount, currentSpeed_mps);
    }
    if (currentSpeed_mps < 0.0 && actuationAmount > 0.0) {
        // Brake +
        return this->reverseBrake(p_doodad, tickTimeDelta, actuationAmount, currentSpeed_mps);
    }

    /** @todo 2025/03/02 Slow down gradually (air brake) */
}

void
PlayerDriver::forwardAcceleration(
    quartz::scene::Doodad* const p_doodad,
    const double tickTimeDelta,
    UNUSED const double actuationAmount,
    UNUSED const double currentSpeed_mps
) {
    const GearInformation& gearInformation = m_gearInformations[m_currentGear];
    const double accelerationRate_mps2 = gearInformation.accelerationRate_mps2;
    const double maxSpeed_kph = gearInformation.maxSpeed_kph;

    // Determine the amount of ticks in a second
    const double tps = 1.0 / tickTimeDelta;

    // meters per second^2 -> meters per tick^2
    /** @todo 2025/02/03 Is this the correct unit to use? */
    // UNUSED const double accelerationRate_mpt2 = accelerationRate_mps2 * (tps * tps);

    // kilometers per hour -> meters per hour -> meters per second
    const double maxSpeed_mps = (maxSpeed_kph * 1000.0) / 60.0;
    
    // kilometers per hour -> kilometers per second -> kilometers per tick
    // UNUSED const double maxSpeed_kpt = (maxSpeed_kph / 60.0) / tps;

    quartz::physics::RigidBody& rigidBody = p_doodad->getRigidBodyOptionalReference().value();
    
    if (currentSpeed_mps >= maxSpeed_mps) {
        return;
    }

    // - we have the current velocity in meters per second
    // - we can calculate what we want the velocity to be in 1 second from now based on our acceleration rate in mps2
    // - we know how many ticks there are from now until 1 second elapses
    // - so we can calculate what we want our velocity to be in 1 tick from now
    // - so we can set the velocity to that value

    // to calculate the velocity delta we should realize in 1 second from now:
    //   - consider the direction we are heading
    //   - multiply the direction's unit vector by (accelerationRate_mps2 * actuationAmount)
    //   - add this resulting vector to our current velocity
    const math::Vec3 velocityDelta_mps_1sFromNow = m_currentDesiredDirection.normalize() * (accelerationRate_mps2 * actuationAmount);

    const math::Vec3 velocityDelta_mpt_1sFromNow = velocityDelta_mps_1sFromNow / tps;

    const math::Vec3 currentLinearVelocity_mps = rigidBody.getLinearVelocity_mps();
    const math::Vec3 currentLinearVelocity_mpt = currentLinearVelocity_mps / tps;
    const math::Vec3 desiredLinearVelocity_mpt_1sFromNow = currentLinearVelocity_mpt + velocityDelta_mpt_1sFromNow;
    const math::Vec3 desiredLinearVelocity_mps_1sFromNow = desiredLinearVelocity_mpt_1sFromNow * tps;

    rigidBody.setLinearVelocity_mps(desiredLinearVelocity_mps_1sFromNow);
}

void
PlayerDriver::forwardBrake(
    UNUSED quartz::scene::Doodad* const p_doodad,
    UNUSED const double tickTimeDelta,
    UNUSED const double actuationAmount,
    UNUSED const double currentSpeed_mps
) {

}

void
PlayerDriver::reverseAcceleration(
    UNUSED quartz::scene::Doodad* const p_doodad,
    UNUSED const double tickTimeDelta,
    UNUSED const double actuationAmount,
    UNUSED const double currentSpeed_mps
) {

}

void
PlayerDriver::reverseBrake(
    UNUSED quartz::scene::Doodad* const p_doodad,
    UNUSED const double tickTimeDelta,
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

