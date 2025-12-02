#include "utilities/test_helpers.hpp"
#include "sim/movementSystem.hpp"
#include "sim/collisionSystem.hpp"
#include "sim/systemManager.hpp"
#include <memory>
#include <iostream>

void test_basic_simulation_step()
{
    // ...
    std::cout << "\n--- TEST: Basic Simulation Step (Controlled Fall) ---\n";

    // 1. Set up a controlled scenario: Body at Y=10.0, zero velocity, gravity -9.8
    body A = create_body(0.0f, 10.0f, 0.0f, 0.0f, 1.0f, 0.5f, 1.0f);
    std::vector<body> bodies = {A};

    // Using a known, controlled delta_time
    float controlled_dt = 0.016f;
    world w;
    w.gravity_x = 0.0f;
    w.gravity_y = -9.8f;
    w.delta_time = controlled_dt;
    for (auto &b : bodies)
        w.add_body(b);

    // This is vital for Verlet: previous_position must be set at the start!
    if (w.num_bodies() > 0)
    {
        w.previous_position_x[0] = w.position_x[0];
        w.previous_position_y[0] = w.position_y[0];
    }

    systemManager manager;
    manager.addSystem(std::make_unique<movementSystem>());
    // ... rest of the code

    manager.update(w, w.delta_time);

    // Expected value after one step: Y ≈ 9.99875.
    std::cout << "First body's position Y after 1 step: " << w.position_y[0] << "\n";
}

void test_world_random_initialization()
{
    test_basic_simulation_step();
}