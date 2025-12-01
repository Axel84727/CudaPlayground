// src/sim/systemManager.cpp (CORREGIDO)

#include "sim/systemManager.hpp"
#include <utility>

void systemManager::addSystem(std::unique_ptr<ISystem> sys)
{

    systems.push_back(std::move(sys));
}

void systemManager::update(world &world, float dt)
{

    for (const auto &system_ptr : systems)
    {

        system_ptr->update(world, dt);
    }
}

systemManager::systemManager() {}
