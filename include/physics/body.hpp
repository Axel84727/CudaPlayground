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
    float restitution = 1.0f;

    body();
    body(const vec2 &position,
         const vec2 &velocity,
         const vec2 &acceleration,
         float mass,
         float inv_mass,
         float radius,
         float restitution = 1.0f);
};