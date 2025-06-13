#pragma once

#include <vector>

#include "quartz/scene/scene/Scene.hpp"

#include "pole_position/third_person_controller/ThirdPersonController.hpp"

enum class PhysicsLayer : uint16_t {
    Default         = 0b0000000000000000,
    Player          = 0b0000000000000001,
    Terrain         = 0b0000000000000010,
    Interactable    = 0b0000000000000100,
};

quartz::scene::Doodad::Parameters
createPlayerDoodadParameters(
    ThirdPersonController& playerController
);

std::vector<quartz::scene::Doodad::Parameters>
createObjectsDoodadParameter();

std::vector<quartz::scene::Doodad::Parameters>
createTerrainDoodadParameter();

quartz::scene::Scene::Parameters
createDemoLevelSceneParameters(
    ThirdPersonController& playerController
);

