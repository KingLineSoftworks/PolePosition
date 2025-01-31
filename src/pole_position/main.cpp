#include <cstdlib>

#include <reactphysics3d/reactphysics3d.h>

#include "util/logger/Logger.hpp"

#include "quartz/application/Application.hpp"

#include "pole_position/Loggers.hpp"
#include "pole_position/Boilerplate.hpp"
#include "pole_position/player_driver/PlayerDriver.hpp"
#include "pole_position/scene/SceneParameters.hpp"

int main() {
    DO_BOILERPLATE(false);

#ifdef QUARTZ_RELEASE
    const bool validationLayersEnabled = false;
#else
    const bool validationLayersEnabled = true;
#endif

    PlayerDriver playerDriver;

    std::vector<quartz::scene::Scene::Parameters> quartzSceneParameters {
        createDemoLevelSceneParameters(playerDriver)
    };

    quartz::Application application(
        APPLICATION_NAME,
        APPLICATION_MAJOR_VERSION,
        APPLICATION_MINOR_VERSION,
        APPLICATION_PATCH_VERSION,
        800,
        600,
        validationLayersEnabled,
        quartzSceneParameters
    );

    try {
        application.run();
    } catch (const std::exception& e) {
        LOG_CRITICAL(GENERAL, "Caught exception");
        LOG_CRITICAL(GENERAL, "{}", e.what());
        return EXIT_FAILURE;
    }

    LOG_TRACE(GENERAL, "Terminating application");

    return EXIT_SUCCESS;
}
