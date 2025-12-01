#pragma once
#include "math/vec2.hpp"

struct body
{
    vec2 position;
    vec2 previous_position;
    vec2 velocity;
    vec2 acceleration;

    float mass;
    float inv_mass;
    float radius;
    // Linear viscous damping coefficient (per-second)
    float damping = 0.0f;
    // Simple linear friction coefficient (Coulomb-like scaled), applied as force opposite velocity
    float friction = 0.0f;
    float restitution = 1.0f;
    // (color removed)

    body();
    body(const vec2 &position,
         const vec2 &velocity,
         const vec2 &acceleration,
         float mass,
         float inv_mass,
         float radius,
         float restitution = 1.0f,
         float damping = 0.0f,
         float friction = 0.0f);
};