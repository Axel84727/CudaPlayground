#pragma once
#include <vector>
#include "physics/body.hpp"
#include "math/vec2.hpp"
struct GridInfo
{
    float min_x = -100.0f;
    float max_x = 100.0f;
    float max_y = 100.0f;
    float min_y = -100.0f;

    const float cell_size = 5.0f;

    int num_cells_x = 0;
    int num_cells_y = 0;
};
struct world
{

    GridInfo grid_info;
    std::vector<body> bodies;

    vec2 gravity_vector;
    float delta_time;

    std::vector<std::vector<body *>> grid;
    world();

    world(const std::vector<body> &b, const vec2 &gravity_vector, float delta_time);

    int get_grid_index(const vec2 &position) const;
};