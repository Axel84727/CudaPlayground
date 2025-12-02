// tests/utilities/test_helpers.hpp

#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include "math/vec2.hpp"
#include "physics/body.hpp"
#include "physics/world.hpp"

// --- Funciones Auxiliares ---

inline void print_vec2(const vec2 &v)
{
    std::cout << "(" << v.x << ", " << v.y << ")";
}

inline body create_body(float pos_x, float pos_y, float vel_x, float vel_y, float masa, float radio, float restitucion = 1.0f)
{
    float inv_mass = (masa > 0.0f) ? 1.0f / masa : 0.0f;
    // default damping, friction for tests
    return body(vec2(pos_x, pos_y), vec2(vel_x, vel_y), vec2(0, 0), masa, inv_mass, radio, restitucion, 0.0f, 0.0f);
}

inline world create_random_world(int num_bodies, const vec2 &gravedad, float dt)
{
    std::vector<body> cuerpos;
    cuerpos.push_back(create_body(0.0f, 10.0f, 0.0f, 0.0f, 1.0f, 0.5f));
    world w;
    w.gravity_x = gravedad.x;
    w.gravity_y = gravedad.y;
    w.delta_time = dt;
    for (auto &b : cuerpos)
        w.add_body(b);
    return w;
}