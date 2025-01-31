#pragma once

#include <vector>

#include "quartz/scene/scene/Scene.hpp"

#include "pole_position/player_driver/PlayerDriver.hpp"

quartz::scene::Scene::Parameters createDemoLevelSceneParameters(
    PlayerDriver& playerDrier
);
