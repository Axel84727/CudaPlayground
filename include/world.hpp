#pragma once
#include <vector>
#include "body.hpp"
#include "vec2.hpp"
struct world
{
    std::vector<body> bodies;
    vec2 gravedad;
    float delta_time;

    world();
    world(const std::vector<body> &b, const vec2 &gravedad, float delta_time);
    void update();
};
