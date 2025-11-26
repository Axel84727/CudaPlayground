#include "body.hpp"
#include "vec2.hpp"
#include "world.hpp"
// world(std::vector<body> &b, const vec2 &gravedad, float delta_time);
world::world() : bodies(), gravedad(), delta_time(0.0f) {}

world::world(const std::vector<body> &b_param, const vec2 &gravedad_param, float delta_time_param) : bodies(b_param), gravedad(gravedad_param), delta_time(delta_time_param)
{

    float width = grid_info.max_x - grid_info.min_x;
    float height = grid_info.max_y - grid_info.min_y;

    int numCellsX = static_cast<int>(std::ceil(width / grid_info.cell_size));
    int numCellsY = static_cast<int>(std::ceil(height / grid_info.cell_size));

    int totalCells = numCellsX * numCellsY;
    grid.resize(totalCells);
}

void world::update()
{
    for (body &b : bodies)
    {
        // 1. Cálculo de Aceleración Total
        vec2 aceleracion_total = b.aceleracion + gravedad;

        // 2. Actualización de Velocidad
        b.velocidad = b.velocidad + (aceleracion_total * delta_time);

        // 3. Actualización de Posición
        b.posicion = b.posicion + (b.velocidad * delta_time);

        // 4. Limpieza
        b.aceleracion = vec2(0.0f, 0.0f);
    }
}
void world::limpieza()
{
    for (auto &cell : grid)
    {
        cell.clear();
    }
}
void world::repoblar()
{
    for (auto &body : bodies)
    {
        int tempIndex = get_grid_index(body.posicion);
        if (tempIndex >= 0)
        {
            grid[tempIndex].push_back(&body);
        }
    }
}