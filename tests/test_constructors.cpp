#include "utilities/test_helpers.hpp"
#include <iostream>

void test_vec2_constructor()
{
    std::cout << "\n--- TEST: Vec2 Constructors ---\n";
    vec2 v;
    print_vec2(v);
    std::cout << " <-- default vec2\n";
    vec2 v2(1.0f, 2.0f);
    print_vec2(v2);
    std::cout << " <-- parameterized vec2\n";
}

void test_body_constructor()
{
    std::cout << "\n--- TEST: Body Constructor ---\n";
    // Using create_body(posX, posY, velX, velY, mass, radius)
    body b = create_body(0, 0, 1, 1, 1, 0.5f);
    print_vec2(b.position); // Accessing the new 'position' member
    std::cout << " | mass: " << b.mass << " | radius: " << b.radius << "\n";
}

void test_world_constructor()
{
    std::cout << "\n--- TEST: World Constructor ---\n";
    std::vector<body> bodies;
    bodies.push_back(create_body(0, 0, 0, 0, 1, 1));
    world w;
    w.gravity_x = 0.0f;
    w.gravity_y = -9.8f;
    w.delta_time = 0.016f;
    for (auto &b : bodies)
        w.add_body(b);
    std::cout << "World bodies size: " << w.size() << "\n";
}

void test_world_constructors()
{
    test_vec2_constructor();
    test_body_constructor();
    test_world_constructor();
}