#include "world.hpp"
#include <cmath>
#include "body.hpp"
#include "vec2.hpp"
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
bool check_collision(body *A, body *B)
{
    vec2 distancia = A->posicion - B->posicion;
    float distancia_al_cuadrado = static_cast<float>(distancia * distancia);
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
int world::get_grid_index(const vec2 &posicion) const
{

    int cx = static_cast<int>(posicion.x / grid_info.cell_size);
    int cy = static_cast<int>(posicion.y / grid_info.cell_size);
    if (cx < 0 || cx >= grid_info.num_cells_x ||
        cy < 0 || cy >= grid_info.num_cells_y)
    {

        return -1;
    }
    int index = cy * grid_info.num_cells_x + cx;

    return index;
}
std::vector<std::pair<body *, body *>> world::broad_phase()
{
    std::vector<std::pair<body *, body *>> pairs;

    for (size_t cellIndex = 0; cellIndex < grid.size(); ++cellIndex)
    {
        auto &currentCell = grid[cellIndex];
        int numCellsX = grid_info.num_cells_x;
        int numCellsY = grid_info.num_cells_y;

        int cy = cellIndex / numCellsX;
        int cx = cellIndex % numCellsX;

        int neighborOffset[3][2] = {
            {1, 0},
            {0, 1},
            {1, 1}};

        for (const auto offset : neighborOffset)
        {
            int dx = offset[0];
            int dy = offset[1];

            int neighborCx = cx + dx;
            int neighborCy = cy + dy;

            if (neighborCx >= numCellsX || neighborCy >= numCellsY)
            {
                continue;
            }

            int neighborIndex = neighborCy * numCellsX + neighborCx;
            auto &neighborCell = grid[neighborIndex];

            for (body *A : currentCell)
            {
                for (body *B : neighborCell)
                {
                    pairs.emplace_back(A, B);
                }
            }
        }

        for (size_t i = 0; i < currentCell.size(); ++i)
        {
            body *A = currentCell[i];
            for (size_t j = i + 1; j < currentCell.size(); ++j)
            {
                body *B = currentCell[j];
                pairs.emplace_back(A, B);
            }
        }
    }

    return pairs;
}