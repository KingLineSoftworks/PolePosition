#include <vector>

#include "math/transform/Vec3.hpp"
#include "pole_position/Loggers.hpp"
#include "quartz/physics/collider/Collider.hpp"
#include "util/file_system/FileSystem.hpp"

#include "quartz/scene/doodad/Doodad.hpp"
#include "quartz/scene/scene/Scene.hpp"

#include "pole_position/scene/SceneParameters.hpp"
#include "pole_position/third_person_controller/ThirdPersonController.hpp"
#include "util/logger/Logger.hpp"

quartz::scene::Doodad::Parameters
createPlayerDoodadParameters(
    ThirdPersonController& playerController
) {
    return {
        util::FileSystem::getAbsoluteFilepathInProjectDirectory("assets/models/unit_models/unit_cube/glb/unit_cube.glb"),
        {
            { 5.0f, 0.5f, 5.0f },
            0.0f,
            { 0.0f, 0.0f, 1.0f },
            { 1.0f, 1.0f, 1.0f }
        },
        {{
            reactphysics3d::BodyType::DYNAMIC,
            true,
            math::Vec3(0.0, 1.0, 0.0),
            {
                false,
                {
                    static_cast<uint16_t>(CollisionCategories::Player),
                    0xFFFF ^ static_cast<uint16_t>(CollisionCategories::Player)
                },
                quartz::physics::BoxShape::Parameters({1.0f, 1.0f, 1.0f}),
                [&playerController] (quartz::physics::Collider::CollisionCallbackParameters parameters) { playerController.collisionStartCallback(parameters); },
                [&playerController] (quartz::physics::Collider::CollisionCallbackParameters parameters) { playerController.collisionStayCallback(parameters); },
                [&playerController] (quartz::physics::Collider::CollisionCallbackParameters parameters) { playerController.collisionEndCallback(parameters); },
            }
        }},
        [&playerController] (quartz::scene::Doodad::AwakenCallbackParameters parameters) { playerController.awakenCallback(parameters); },
        [&playerController] (quartz::scene::Doodad::FixedUpdateCallbackParameters parameters) { playerController.fixedUpdateCallback(parameters); },
        [&playerController] (quartz::scene::Doodad::UpdateCallbackParameters parameters) { playerController.updateCallback(parameters); }
    };
}

std::vector<quartz::scene::Doodad::Parameters>
createObjectsDoodadParameter() {
    return {
        // The water bottle 
        {
            util::FileSystem::getAbsoluteFilepathInProjectDirectory("assets/models/glTF-Sample-Models/2.0/WaterBottle/glTF-Binary/WaterBottle.glb"),
            {
                { 10.0f, 3.0f, 10.0f },
                0.0f,
                { 0.0f, 0.0f, 1.0f },
                { 10.0f, 10.0f, 10.0f }
            },
            {{
                reactphysics3d::BodyType::STATIC,
                true,
                math::Vec3(0.0, 1.0, 0.0),
                {
                    false,
                    {
                        static_cast<uint16_t>(CollisionCategories::Interactable),
                        0xFFFF
                    },
                    quartz::physics::BoxShape::Parameters({1.0f, 1.0f, 1.0f}),
                    [&] (UNUSED quartz::physics::Collider::CollisionCallbackParameters parameters) { },
                    [&] (UNUSED quartz::physics::Collider::CollisionCallbackParameters parameters) { },
                    [&] (UNUSED quartz::physics::Collider::CollisionCallbackParameters parameters) { }
                }
            }},
            [&] (UNUSED quartz::scene::Doodad::AwakenCallbackParameters parameters) { },
            [&] (UNUSED quartz::scene::Doodad::FixedUpdateCallbackParameters parameters) { },
            [&] (UNUSED quartz::scene::Doodad::UpdateCallbackParameters parameters) { }
        },

        // The boombox
        {
            util::FileSystem::getAbsoluteFilepathInProjectDirectory("assets/models/glTF-Sample-Models/2.0/BoomBox/glTF-Binary/BoomBox.glb"),
            {
                { 20.0f, 3.0f, 20.0f },
                0.0f,
                { 0.0f, 0.0f, 1.0f },
                { 200.0f, 200.0f, 200.0f }
            },
            {{
                reactphysics3d::BodyType::STATIC,
                true,
                math::Vec3(0.0, 1.0, 0.0),
                {
                    true,
                    {
                        static_cast<uint16_t>(CollisionCategories::Interactable),
                        0xFFFF
                    },
                    quartz::physics::BoxShape::Parameters({1.0f, 1.0f, 1.0f}),
                    [&] (UNUSED quartz::physics::Collider::CollisionCallbackParameters parameters) { },
                    [&] (UNUSED quartz::physics::Collider::CollisionCallbackParameters parameters) { },
                    [&] (UNUSED quartz::physics::Collider::CollisionCallbackParameters parameters) { }
                }
            }},
            [&] (UNUSED quartz::scene::Doodad::AwakenCallbackParameters parameters) { },
            [&] (UNUSED quartz::scene::Doodad::FixedUpdateCallbackParameters parameters) { },
            [&] (UNUSED quartz::scene::Doodad::UpdateCallbackParameters parameters) { }
        },
    };
}

std::vector<quartz::scene::Doodad::Parameters>
createTerrainDoodadParameter() {
    return {
        // The ground bro
        {
            util::FileSystem::getAbsoluteFilepathInProjectDirectory("assets/models/glTF-Sample-Models/2.0/Cube/glTF/Cube.gltf"),
            {
                {0.0f, -0.5f, 0.0f},
                0.0f,
                {0.0f, 1.0f, 0.0f},
                {200.0f, 1.0f, 200.0f}
            },
            {{
                reactphysics3d::BodyType::STATIC,
                false,
                math::Vec3(1.0, 1.0, 1.0),
                {
                    false,
                    {
                        static_cast<uint16_t>(CollisionCategories::Terrain),
                        0xFFFF ^ static_cast<uint16_t>(CollisionCategories::Terrain)
                    },
                    quartz::physics::BoxShape::Parameters({200.0f, 1.0f, 200.0f}),
                    {},
                    {},
                    {}
                }
            }},
            {},
            {},
            {}
        }
    };
}

quartz::scene::Scene::Parameters
createDemoLevelSceneParameters(
    ThirdPersonController& playerController
) {
    std::vector<quartz::scene::Doodad::Parameters> objectsDoodadParameters = createObjectsDoodadParameter();
    std::vector<quartz::scene::Doodad::Parameters> terrainDoodadParameters = createTerrainDoodadParameter();
    std::vector<quartz::scene::Doodad::Parameters> doodadParameters = { createPlayerDoodadParameters(playerController) };
    doodadParameters.reserve(doodadParameters.size() + objectsDoodadParameters.size() + terrainDoodadParameters.size());
    doodadParameters.insert(doodadParameters.end(), objectsDoodadParameters.begin(), objectsDoodadParameters.end());
    doodadParameters.insert(doodadParameters.end(), terrainDoodadParameters.begin(), terrainDoodadParameters.end());

    quartz::scene::AmbientLight ambientLight({ 0.1f, 0.1f, 0.1f });

    quartz::scene::DirectionalLight directionalLight({ 0.5f, 0.5f, 0.5f }, { 3.0f, -2.0f, 2.0f });

    std::vector<quartz::scene::PointLight> pointLights = {};

    std::vector<quartz::scene::SpotLight> spotLights = {};

    math::Vec3 screenClearColor = { 0.25f, 0.4f, 0.6f };
    
    std::array<std::string, 6> skyBoxInformation = {
        util::FileSystem::getAbsoluteFilepathInProjectDirectory("assets/sky_boxes/parliament/posx.jpg"),
        util::FileSystem::getAbsoluteFilepathInProjectDirectory("assets/sky_boxes/parliament/negx.jpg"),
        util::FileSystem::getAbsoluteFilepathInProjectDirectory("assets/sky_boxes/parliament/posy.jpg"),
        util::FileSystem::getAbsoluteFilepathInProjectDirectory("assets/sky_boxes/parliament/negy.jpg"),
        util::FileSystem::getAbsoluteFilepathInProjectDirectory("assets/sky_boxes/parliament/posz.jpg"),
        util::FileSystem::getAbsoluteFilepathInProjectDirectory("assets/sky_boxes/parliament/negz.jpg")
    };

    std::optional<quartz::physics::Field::Parameters> o_fieldParameters({{0.0, -1.0, 0.0}});

    return { 
        "default_test_scene_00",
        ambientLight,
        directionalLight,
        pointLights,
        spotLights,
        screenClearColor,
        skyBoxInformation,
        doodadParameters,
        o_fieldParameters
    };
}

