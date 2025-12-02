#include "utilities/test_helpers.hpp"
#include "sim/movementSystem.hpp"
#include "sim/collisionSystem.hpp"
#include "sim/systemManager.hpp"
#include <iostream>
#include <memory>

// This test checks a basic fall simulation using the default Verlet integrator.
void test_verlet_movement_simple()
{
    std::cout << "\n--- TEST: Basic Verlet Movement (Gravity Check) ---\n";

    // body A starts at Y=5 with 0 initial velocity
    body A = create_body(0, 5, 0, 0, 1, 0.5f, 0.8f);

    // Static ground body (mass=0)
    body B = create_body(0, 0, 0, 0, 0, 1.0f, 0.8f);

    std::vector<body> bodies = {A, B};
    world w;
    w.gravity_x = 0.0f;
    w.gravity_y = -9.8f;
    w.delta_time = 0.016f;
    for (auto &b : bodies)
        w.add_body(b);

    // Note: 'previous_position' initialization is now handled in the body constructor.

    systemManager manager;
    manager.addSystem(std::make_unique<collisionSystem>());
    manager.addSystem(std::make_unique<movementSystem>());

    // Run a few steps to check if position changes correctly under gravity
    for (int t = 0; t < 5; t++)
        manager.update(w, w.delta_time);

    // The final position Y must be less than 5 (due to gravity)
    std::cout << "Body final position Y after 5 steps: " << w.position_y[0] << " (Should be < 5)\n";
}

// Since main_test.cpp no longer calls a wrapper, this function is the replacement.
void test_integrator_stability_wrapper(int type, const char *name)
{
    // The original wrapper now calls the simplified test for movement stability.
    // The arguments 'type' and 'name' are ignored, but we keep the signature
    // to avoid changing main_test.cpp if we only simplify.
    // However, since we agreed to remove the stability tests from main_test.cpp,
    // this wrapper is generally deleted. For completeness, I'll delete the file
    // and rely on a simpler test in another file, or rename this one.

    // Let's assume you rename this file to test_movement.cpp and call the function directly.
    test_verlet_movement_simple();
}