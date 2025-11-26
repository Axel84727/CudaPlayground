#pragma once
#include <vector>
#include "body.hpp"
#include "vec2.hpp"
// TODO: repoblar las matrices luego de las limpiezas
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
    vec2 gravedad;
    float delta_time;

    std::vector<std::vector<body *>> grid;
    world();
    // limpiar el vector
    void world::limpieza();
    world(const std::vector<body> &b, const vec2 &gravedad, float delta_time);
    // ticks del mundo
    void world::update();
    // repoblar el vector
    void world::repoblar();
    // conseguir el indice
    int world::get_grid_index(const vec2 &posicion) const;
    void world::broad_phase();
};
