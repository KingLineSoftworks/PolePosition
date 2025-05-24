#pragma once

#include "quartz/scene/scene/Scene.hpp"

#include "pole_position/third_person_controller/ThirdPersonController.hpp"

quartz::scene::Scene::Parameters createDemoLevelSceneParameters(
    ThirdPersonController& playerDrier
);
