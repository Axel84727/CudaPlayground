#include "sim/collisionSystem.hpp"
#include "physics/world.hpp"
#include "physics/body.hpp"
#include "math/vec2.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>

// ====================================================================
// --- TUNING CONFIGURATION (Move to a header or settings) ---
// ====================================================================
// It's recommended to move these values to private members or a configuration class.
const float POSITION_CORRECTION_SLOP = 0.001f;  // Minimum penetration before correcting
const float POSITION_CORRECTION_PERCENT = 0.2f; // Percentage of penetration to correct (smaller to avoid energy loss)
const float VELOCITY_EPSILON = 1e-6f;           // Threshold to snap velocity to zero (smaller to avoid early sleeping)

// ====================================================================
// --- CONSTRUCTOR/DESTRUCTOR ---
// ====================================================================

collisionSystem::collisionSystem() {}
collisionSystem::~collisionSystem() {}

// ====================================================================
// --- GRID PHASES (Spatial Hashing) ---
// ====================================================================

void collisionSystem::clear_spatial_grid(world &simulation_world)
{
    for (auto &cell_body_list : simulation_world.grid)
    {
        cell_body_list.clear();
    }
}

void collisionSystem::populate_spatial_grid(world &simulation_world)
{
    size_t n = simulation_world.position_x.size();
    for (size_t i = 0; i < n; ++i)
    {
        vec2 pos(simulation_world.position_x[i], simulation_world.position_y[i]);
        int grid_index = simulation_world.get_grid_index(pos);
        if (grid_index >= 0)
        {
            simulation_world.grid[grid_index].push_back((int)i);
        }
    }
}

// ====================================================================
// --- BROAD PHASE: Generate Candidate Pairs ---
// ====================================================================

std::vector<std::pair<int, int>> collisionSystem::broad_phase_generate_pairs(world &simulation_world)
{
    std::vector<std::pair<int, int>> potential_collision_pairs;

    int num_cells_x = simulation_world.grid_info.num_cells_x;
    int num_cells_y = simulation_world.grid_info.num_cells_y;

    // Neighbor offsets: only check right, down, and down-right to avoid duplicates
    const int neighbor_offsets[3][2] = {
        {1, 0}, // Right
        {0, 1}, // Down
        {1, 1}  // Down-Right
    };

    for (size_t cell_index = 0; cell_index < simulation_world.grid.size(); ++cell_index)
    {
        auto &current_cell_bodies = simulation_world.grid[cell_index];

        int current_cell_y = cell_index / num_cells_x;
        int current_cell_x = cell_index % num_cells_x;

        // 1. Check against neighbor cells
        for (const auto &offset : neighbor_offsets)
        {
            int offset_x = offset[0];
            int offset_y = offset[1];

            int neighbor_cell_x = current_cell_x + offset_x;
            int neighbor_cell_y = current_cell_y + offset_y;

            if (neighbor_cell_x >= num_cells_x || neighbor_cell_y >= num_cells_y)
            {
                continue;
            }

            int neighbor_index = neighbor_cell_y * num_cells_x + neighbor_cell_x;
            auto &neighbor_cell_bodies = simulation_world.grid[neighbor_index];

            for (int idxA : current_cell_bodies)
            {
                for (int idxB : neighbor_cell_bodies)
                {
                    potential_collision_pairs.emplace_back(idxA, idxB);
                }
            }
        }

        // 2. Check within the same cell
        for (size_t i = 0; i < current_cell_bodies.size(); ++i)
        {
            int idxA = current_cell_bodies[i];
            for (size_t j = i + 1; j < current_cell_bodies.size(); ++j)
            {
                int idxB = current_cell_bodies[j];
                potential_collision_pairs.emplace_back(idxA, idxB);
            }
        }
    }

    return potential_collision_pairs;
}

// ====================================================================
// --- CIRCLE-CIRCLE CHECK (Narrow Phase Detection) ---
// ====================================================================

bool collisionSystem::check_for_overlap(int idxA, int idxB, world &simulation_world)
{
    vec2 a(simulation_world.position_x[idxA], simulation_world.position_y[idxA]);
    vec2 b(simulation_world.position_x[idxB], simulation_world.position_y[idxB]);
    vec2 displacement_vector = a - b;
    float distance_squared = dot(displacement_vector, displacement_vector); // Avoid sqrt()

    float sum_of_radii = simulation_world.radius[idxA] + simulation_world.radius[idxB];
    float sum_of_radii_squared = sum_of_radii * sum_of_radii;

    return distance_squared <= sum_of_radii_squared;
}

// ====================================================================
// --- NARROW PHASE: Check and Resolve ---
// ====================================================================

void collisionSystem::narrow_phase_check_and_resolve(world &simulation_world)
{
    // Broad phase timing
    auto t_b0 = std::chrono::high_resolution_clock::now();
    auto potential_pairs = broad_phase_generate_pairs(simulation_world);
    auto t_b1 = std::chrono::high_resolution_clock::now();
    auto broad_us = std::chrono::duration_cast<std::chrono::microseconds>(t_b1 - t_b0).count();
    simulation_world.broad_phase_us = (unsigned long long)broad_us;

    // Narrow phase timing
    auto t_n0 = std::chrono::high_resolution_clock::now();
    for (auto &[idxA, idxB] : potential_pairs)
    {
        float invA = simulation_world.inv_mass[idxA];
        float invB = simulation_world.inv_mass[idxB];
        if (invA == 0.0f && invB == 0.0f)
            continue;

        if (check_for_overlap(idxA, idxB, simulation_world))
        {
            auto t_r0 = std::chrono::high_resolution_clock::now();
            resolve_contact_with_impulse(idxA, idxB, simulation_world);
            auto t_r1 = std::chrono::high_resolution_clock::now();
            auto resolve_us = std::chrono::duration_cast<std::chrono::microseconds>(t_r1 - t_r0).count();
            simulation_world.resolve_phase_us += (unsigned long long)resolve_us;
        }
    }
    auto t_n1 = std::chrono::high_resolution_clock::now();
    auto narrow_us = std::chrono::duration_cast<std::chrono::microseconds>(t_n1 - t_n0).count();
    simulation_world.narrow_phase_us = (unsigned long long)narrow_us;
}

// ====================================================================
// --- CONTACT RESOLUTION (Impulse and Position Correction) ---
// ====================================================================

void collisionSystem::resolve_contact_with_impulse(int idxA, int idxB, world &simulation_world)
{
    vec2 posA(simulation_world.position_x[idxA], simulation_world.position_y[idxA]);
    vec2 posB(simulation_world.position_x[idxB], simulation_world.position_y[idxB]);
    vec2 displacement_vector = posB - posA;
    float distance_squared = dot(displacement_vector, displacement_vector);

    if (distance_squared <= 1e-6f)
        return;

    float distance = std::sqrt(distance_squared);
    float sum_of_radii = simulation_world.radius[idxA] + simulation_world.radius[idxB];
    float penetration_depth = sum_of_radii - distance;
    if (penetration_depth <= 0.0f)
        return;

    vec2 collision_normal = displacement_vector * (1.0f / distance);

    float inverse_mass_A = simulation_world.inv_mass[idxA];
    float inverse_mass_B = simulation_world.inv_mass[idxB];
    float inverse_mass_sum = inverse_mass_A + inverse_mass_B;
    if (inverse_mass_sum <= 0.0f)
        return;

    float correction_magnitude = std::max(penetration_depth - POSITION_CORRECTION_SLOP, 0.0f) / inverse_mass_sum * POSITION_CORRECTION_PERCENT;
    vec2 position_correction_vector = collision_normal * correction_magnitude;

    // Apply correction to SoA positions
    simulation_world.position_x[idxA] -= position_correction_vector.x * inverse_mass_A;
    simulation_world.position_y[idxA] -= position_correction_vector.y * inverse_mass_A;
    simulation_world.position_x[idxB] += position_correction_vector.x * inverse_mass_B;
    simulation_world.position_y[idxB] += position_correction_vector.y * inverse_mass_B;

    // velocities
    vec2 velA(simulation_world.vel_x[idxA], simulation_world.vel_y[idxA]);
    vec2 velB(simulation_world.vel_x[idxB], simulation_world.vel_y[idxB]);
    vec2 relative_velocity = velB - velA;
    float velocity_along_normal = dot(relative_velocity, collision_normal);
    if (velocity_along_normal > 0.0f)
        return;

    float effective_restitution = (simulation_world.get_restitution(idxA) + simulation_world.get_restitution(idxB)) * 0.5f;

    float impulse_scalar = -(1.0f + effective_restitution) * velocity_along_normal;
    impulse_scalar /= inverse_mass_sum;
    vec2 collision_impulse_vector = collision_normal * impulse_scalar;

    velA = velA - collision_impulse_vector * inverse_mass_A;
    velB = velB + collision_impulse_vector * inverse_mass_B;

    simulation_world.vel_x[idxA] = velA.x;
    simulation_world.vel_y[idxA] = velA.y;
    simulation_world.vel_x[idxB] = velB.x;
    simulation_world.vel_y[idxB] = velB.y;

    // Update previous positions for Verlet consistency
    float dt = simulation_world.delta_time;
    if (dt > 0.0f)
    {
        simulation_world.previous_position_x[idxA] = simulation_world.position_x[idxA] - velA.x * dt;
        simulation_world.previous_position_y[idxA] = simulation_world.position_y[idxA] - velA.y * dt;
        simulation_world.previous_position_x[idxB] = simulation_world.position_x[idxB] - velB.x * dt;
        simulation_world.previous_position_y[idxB] = simulation_world.position_y[idxB] - velB.y * dt;
    }

    // Ensure positions are nudged slightly outward to avoid exact-contact re-penetration
    const float BOUNDARY_EPS = 1e-4f;
    // clamp A
    float min_x = simulation_world.grid_info.min_x;
    float max_x = simulation_world.grid_info.max_x;
    float min_y = simulation_world.grid_info.min_y;
    float max_y = simulation_world.grid_info.max_y;
    float rA = simulation_world.radius[idxA];
    float rB = simulation_world.radius[idxB];
    simulation_world.position_x[idxA] = std::min(std::max(simulation_world.position_x[idxA], min_x + rA + BOUNDARY_EPS), max_x - rA - BOUNDARY_EPS);
    simulation_world.position_y[idxA] = std::min(std::max(simulation_world.position_y[idxA], min_y + rA + BOUNDARY_EPS), max_y - rA - BOUNDARY_EPS);
    simulation_world.position_x[idxB] = std::min(std::max(simulation_world.position_x[idxB], min_x + rB + BOUNDARY_EPS), max_x - rB - BOUNDARY_EPS);
    simulation_world.position_y[idxB] = std::min(std::max(simulation_world.position_y[idxB], min_y + rB + BOUNDARY_EPS), max_y - rB - BOUNDARY_EPS);

    // 4. LOW-VELOCITY ELIMINATION (Sleeping) - operate on SoA velocities
    if (std::fabs(simulation_world.vel_x[idxA]) < VELOCITY_EPSILON)
        simulation_world.vel_x[idxA] = 0.0f;
    if (std::fabs(simulation_world.vel_y[idxA]) < VELOCITY_EPSILON)
        simulation_world.vel_y[idxA] = 0.0f;
    if (std::fabs(simulation_world.vel_x[idxB]) < VELOCITY_EPSILON)
        simulation_world.vel_x[idxB] = 0.0f;
    if (std::fabs(simulation_world.vel_y[idxB]) < VELOCITY_EPSILON)
        simulation_world.vel_y[idxB] = 0.0f;
}

// ====================================================================
// --- WORLD BOUNDARY (Boundary) ---
// ====================================================================

void collisionSystem::solve_boundary_contacts(world &simulation_world)
{
    size_t n = simulation_world.position_x.size();
    float min_x = simulation_world.grid_info.min_x;
    float max_x = simulation_world.grid_info.max_x;
    float min_y = simulation_world.grid_info.min_y;
    float max_y = simulation_world.grid_info.max_y;
    const float ground_y_limit = 0.0f;

    for (size_t i = 0; i < n; ++i)
    {
        if (simulation_world.inv_mass[i] == 0.0f)
            continue;

        float px = simulation_world.position_x[i];
        float py = simulation_world.position_y[i];
        float vx = simulation_world.vel_x[i];
        float vy = simulation_world.vel_y[i];
        float r = simulation_world.radius[i];
        float restitution = simulation_world.get_restitution(i);

        if (py - r < ground_y_limit)
        {
            py = ground_y_limit + r;
            if (vy < 0.0f)
                vy = -vy * restitution;
        }

        if (px - r < min_x)
        {
            px = min_x + r;
            if (vx < 0.0f)
                vx = -vx * restitution;
        }

        if (px + r > max_x)
        {
            px = max_x - r;
            if (vx > 0.0f)
                vx = -vx * restitution;
        }

        if (py + r > max_y)
        {
            py = max_y - r;
            if (vy > 0.0f)
                vy = -vy * restitution;
        }

        if (std::fabs(vx) < VELOCITY_EPSILON)
            vx = 0.0f;
        if (std::fabs(vy) < VELOCITY_EPSILON)
            vy = 0.0f;

        simulation_world.position_x[i] = px;
        simulation_world.position_y[i] = py;
        simulation_world.vel_x[i] = vx;
        simulation_world.vel_y[i] = vy;

        float dt = simulation_world.delta_time;
        if (dt > 0.0f)
        {
            simulation_world.previous_position_x[i] = px - vx * dt;
            simulation_world.previous_position_y[i] = py - vy * dt;
        }
        // small inward nudge to avoid exact contact with boundaries which can cause
        // re-penetration or sticky behavior due to floating point rounding.
        const float NUDGE = 1e-4f;
        simulation_world.position_x[i] = std::min(std::max(simulation_world.position_x[i], min_x + r + NUDGE), max_x - r - NUDGE);
        simulation_world.position_y[i] = std::min(std::max(simulation_world.position_y[i], min_y + r + NUDGE), max_y - r - NUDGE);
        // SoA arrays are canonical.
    }
}

// ====================================================================
// --- MAIN UPDATE LOOP ---
// ====================================================================

void collisionSystem::update(world &simulation_world, float delta_time)
{
    // 1. Preparation phase (Spatial Hashing)
    clear_spatial_grid(simulation_world);
    populate_spatial_grid(simulation_world);

    // 2. Body-Body collisions (Broad and Narrow Phase)
    narrow_phase_check_and_resolve(simulation_world);

    // 3. World boundary collisions
    solve_boundary_contacts(simulation_world);
}