#include "physics/world.hpp"
#include "physics/body.hpp"
#include <utility>
#include <cmath>
#include <iostream>

world::world() : gravity_x(0.0f),
                 gravity_y(-41.63f),
                 delta_time(1.0f / 60.0f)
{
}

world::world(
    std::vector<float> &&position_x_in,
    std::vector<float> &&position_y_in,
    float gravity_x_in,
    float gravity_y_in,
    float delta_time_in,

    std::vector<int> &&particle_cell_id_in,
    std::vector<int> &&particle_start_indices_in,
    std::vector<int> &&sorted_indices_in,

    std::vector<float> &&vel_x_in,
    std::vector<float> &&vel_y_in,
    std::vector<float> &&acc_x_in,
    std::vector<float> &&acc_y_in,
    std::vector<float> &&mass_in,
    std::vector<float> &&inv_mass_in,
    std::vector<float> &&radius_in)

    : position_x(std::move(position_x_in)),
      position_y(std::move(position_y_in)),
      gravity_x(gravity_x_in),
      gravity_y(gravity_y_in),
      delta_time(delta_time_in),

      particle_cell_id(std::move(particle_cell_id_in)),
      particle_start_indices(std::move(particle_start_indices_in)),
      sorted_indices(std::move(sorted_indices_in)),

      vel_x(std::move(vel_x_in)),
      vel_y(std::move(vel_y_in)),
      acc_x(std::move(acc_x_in)),
      acc_y(std::move(acc_y_in)),
      mass(std::move(mass_in)),
      inv_mass(std::move(inv_mass_in)),
      radius(std::move(radius_in))
{

    float width = grid_info.max_x - grid_info.min_x;
    float height = grid_info.max_y - grid_info.min_y;

    int numCellsX = static_cast<int>(std::ceil(width / grid_info.cell_size));
    int numCellsY = static_cast<int>(std::ceil(height / grid_info.cell_size));

    grid_info.num_cells_x = numCellsX;
    grid_info.num_cells_y = numCellsY;

    int totalCells = numCellsX * numCellsY;
    particle_start_indices.resize(totalCells);
    grid.clear();
    grid.resize(totalCells);
}

// SoA constructor: accept position arrays (by copy). Other arrays can be populated later.
world::world(const std::vector<float> &position_x_in, const std::vector<float> &position_y_in, const vec2 &gravity_vec, float delta_time_in)
    : position_x(position_x_in), position_y(position_y_in), gravity_x(gravity_vec.x), gravity_y(gravity_vec.y), delta_time(delta_time_in)
{
    size_t n = position_x.size();
    previous_position_x.resize(n);
    previous_position_y.resize(n);
    vel_x.resize(n);
    vel_y.resize(n);
    acc_x.resize(n);
    acc_y.resize(n);
    mass.resize(n);
    inv_mass.resize(n);
    radius.resize(n);
    damping.resize(n);
    friction.resize(n);
    restitution.resize(n);

    // initialize previous positions to current positions
    for (size_t i = 0; i < n; ++i)
    {
        previous_position_x[i] = position_x[i];
        previous_position_y[i] = position_y[i];
    }

    float width = grid_info.max_x - grid_info.min_x;
    float height = grid_info.max_y - grid_info.min_y;

    int numCellsX = static_cast<int>(std::ceil(width / grid_info.cell_size));
    int numCellsY = static_cast<int>(std::ceil(height / grid_info.cell_size));

    grid_info.num_cells_x = numCellsX;
    grid_info.num_cells_y = numCellsY;

    int totalCells = numCellsX * numCellsY;
    particle_start_indices.resize(totalCells);
    grid.clear();
    grid.resize(totalCells);
}

void world::add_body(const body &b)
{
    position_x.push_back(b.position.x);
    position_y.push_back(b.position.y);
    previous_position_x.push_back(b.previous_position.x);
    previous_position_y.push_back(b.previous_position.y);
    vel_x.push_back(b.velocity.x);
    vel_y.push_back(b.velocity.y);
    acc_x.push_back(b.acceleration.x);
    acc_y.push_back(b.acceleration.y);
    mass.push_back(b.mass);
    inv_mass.push_back(b.inv_mass);
    radius.push_back(b.radius);
    damping.push_back(b.damping);
    friction.push_back(b.friction);
    restitution.push_back(b.restitution);
}

void world::remove_body(size_t idx)
{
    if (idx >= position_x.size())
        return;
    // swap-remove to keep O(1)
    size_t last = position_x.size() - 1;
    if (idx != last)
    {
        position_x[idx] = position_x[last];
        position_y[idx] = position_y[last];
        vel_x[idx] = vel_x[last];
        vel_y[idx] = vel_y[last];
        acc_x[idx] = acc_x[last];
        acc_y[idx] = acc_y[last];
        mass[idx] = mass[last];
        inv_mass[idx] = inv_mass[last];
        radius[idx] = radius[last];
        damping[idx] = damping[last];
        friction[idx] = friction[last];
        restitution[idx] = restitution[last];
    }
    position_x.pop_back();
    position_y.pop_back();
    vel_x.pop_back();
    vel_y.pop_back();
    acc_x.pop_back();
    acc_y.pop_back();
    mass.pop_back();
    inv_mass.pop_back();
    radius.pop_back();
    damping.pop_back();
    friction.pop_back();
    restitution.pop_back();
}

vec2 world::get_position(size_t idx) const
{
    if (idx >= position_x.size())
        return vec2(0, 0);
    return vec2(position_x[idx], position_y[idx]);
}

void world::set_position(size_t idx, const vec2 &p)
{
    if (idx >= position_x.size())
        return;
    position_x[idx] = p.x;
    position_y[idx] = p.y;
    // Keep previous_position consistent to avoid large implied velocities
    if (idx < previous_position_x.size())
    {
        previous_position_x[idx] = p.x;
        previous_position_y[idx] = p.y;
    }
    // Zero velocity when position is explicitly set through the API
    if (idx < vel_x.size())
    {
        vel_x[idx] = 0.0f;
        vel_y[idx] = 0.0f;
    }
}

// Legacy conversion helpers removed; no legacy definitions remain here.
float world::get_restitution(size_t idx) const
{
    if (idx < restitution.size())
        return restitution[idx];
    return 1.0f;
}

float world::get_damping(size_t idx) const
{
    if (idx < damping.size())
        return damping[idx];
    return 0.0f;
}

float world::get_friction(size_t idx) const
{
    if (idx < friction.size())
        return friction[idx];
    return 0.0f;
}

int world::get_grid_index(const vec2 &posicion) const
{
    int cx = static_cast<int>((posicion.x - grid_info.min_x) / grid_info.cell_size);
    int cy = static_cast<int>((posicion.y - grid_info.min_y) / grid_info.cell_size);

    if (cx < 0 || cx >= grid_info.num_cells_x ||
        cy < 0 || cy >= grid_info.num_cells_y)
    {
        return -1;
    }
    int index = cy * grid_info.num_cells_x + cx;

    return index;
}
