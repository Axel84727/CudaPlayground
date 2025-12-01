// include/sim/systemManager.hpp (CORREGIDO)

#pragma once

#include "sim/ISystem.hpp"
#include <vector>
#include <memory>

class world;

class systemManager
{
private:
    std::vector<std::unique_ptr<ISystem>> systems;

public:
    void addSystem(std::unique_ptr<ISystem> sys);

    void update(world &world, float dt);

    systemManager();
    ~systemManager() = default;
};