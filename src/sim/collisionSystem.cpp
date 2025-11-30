#include "sim/collisionSystem.hpp"
#include "physics/world.hpp"
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
const float POSITION_CORRECTION_PERCENT = 0.4f; // Percentage of penetration to correct
const float VELOCITY_EPSILON = 1e-3f;           // Threshold to snap velocity to zero

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
    for (auto &current_body : simulation_world.bodies)
    {
        int grid_index = simulation_world.get_grid_index(current_body.position);
        if (grid_index >= 0)
        {
            simulation_world.grid[grid_index].push_back(&current_body);
        }
    }
}

// ====================================================================
// --- BROAD PHASE: Generate Candidate Pairs ---
// ====================================================================

std::vector<std::pair<body *, body *>> collisionSystem::broad_phase_generate_pairs(world &simulation_world)
{
    std::vector<std::pair<body *, body *>> potential_collision_pairs;

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

            for (body *body_A : current_cell_bodies)
            {
                for (body *body_B : neighbor_cell_bodies)
                {
                    potential_collision_pairs.emplace_back(body_A, body_B);
                }
            }
        }

        // 2. Check within the same cell
        for (size_t i = 0; i < current_cell_bodies.size(); ++i)
        {
            body *body_A = current_cell_bodies[i];
            for (size_t j = i + 1; j < current_cell_bodies.size(); ++j)
            {
                body *body_B = current_cell_bodies[j];
                potential_collision_pairs.emplace_back(body_A, body_B);
            }
        }
    }

    return potential_collision_pairs;
}

// ====================================================================
// --- CIRCLE-CIRCLE CHECK (Narrow Phase Detection) ---
// ====================================================================

bool collisionSystem::check_for_overlap(body *body_A, body *body_B, world &simulation_world)
{
    vec2 displacement_vector = body_A->position - body_B->position;
    float distance_squared = dot(displacement_vector, displacement_vector); // Avoid sqrt()

    float sum_of_radii = body_A->radius + body_B->radius;
    float sum_of_radii_squared = sum_of_radii * sum_of_radii;

    return distance_squared <= sum_of_radii_squared;
}

// ====================================================================
// --- NARROW PHASE: Check and Resolve ---
// ====================================================================

void collisionSystem::narrow_phase_check_and_resolve(world &simulation_world)
{
    auto potential_pairs = broad_phase_generate_pairs(simulation_world);

    for (auto &[body_A, body_B] : potential_pairs)
    {
        // If both bodies are static (infinite mass), skip resolution
        if (body_A->inv_mass == 0.0f && body_B->inv_mass == 0.0f)
        {
            continue;
        }

        if (check_for_overlap(body_A, body_B, simulation_world))
        {
            resolve_contact_with_impulse(body_A, body_B, simulation_world);
        }
    }
}

// ====================================================================
// --- CONTACT RESOLUTION (Impulse and Position Correction) ---
// ====================================================================

void collisionSystem::resolve_contact_with_impulse(body *body_A, body *body_B, world &simulation_world)
{
    // 1. CONTACT DATA CALCULATION (Manifold)
    vec2 displacement_vector = body_B->position - body_A->position;
    float distance_squared = dot(displacement_vector, displacement_vector);

    if (distance_squared <= 1e-6f)
        return;

    float distance = std::sqrt(distance_squared);
    float sum_of_radii = body_A->radius + body_B->radius;
    float penetration_depth = sum_of_radii - distance;

    if (penetration_depth <= 0.0f)
        return;

    vec2 collision_normal = displacement_vector * (1.0f / distance);

    float inverse_mass_A = body_A->inv_mass;
    float inverse_mass_B = body_B->inv_mass;
    float inverse_mass_sum = inverse_mass_A + inverse_mass_B;

    if (inverse_mass_sum <= 0.0f)
        return;

    // 2. POSITION CORRECTION (Separate the bodies)

    // Calculate correction, applying "slop" margin for stability
    float correction_magnitude = std::max(penetration_depth - POSITION_CORRECTION_SLOP, 0.0f) / inverse_mass_sum * POSITION_CORRECTION_PERCENT;

    vec2 position_correction_vector = collision_normal * correction_magnitude;

    // Apply correction proportional to inverse mass
    body_A->position = body_A->position - position_correction_vector * inverse_mass_A;
    body_B->position = body_B->position + position_correction_vector * inverse_mass_B;

    // 3. IMPULSE RESOLUTION (Change of Velocity)

    vec2 relative_velocity = body_B->velocity - body_A->velocity;
    float velocity_along_normal = dot(relative_velocity, collision_normal);

    // If bodies are already separating, don't apply impulse
    if (velocity_along_normal > 0.0f)
        return;

    float effective_restitution = (body_A->restitution + body_B->restitution) * 0.5f;

    // Compute scalar impulse (J)
    float impulse_scalar = -(1.0f + effective_restitution) * velocity_along_normal;
    impulse_scalar /= inverse_mass_sum;

    // J is positive, and collision_normal points from A to B
    vec2 collision_impulse_vector = collision_normal * impulse_scalar;

    // Apply impulse:
    // Body A: Pushed AWAY from the collision (vector impulse points away from B)
    body_A->velocity = body_A->velocity - collision_impulse_vector * inverse_mass_A;

    // Body B: Pushed ALONG the collision normal (vector impulse points away from A)
    body_B->velocity = body_B->velocity + collision_impulse_vector * inverse_mass_B;

    // 4. LOW-VELOCITY ELIMINATION (Sleeping)
    if (std::fabs(body_A->velocity.x) < VELOCITY_EPSILON)
        body_A->velocity.x = 0.0f;
    if (std::fabs(body_A->velocity.y) < VELOCITY_EPSILON)
        body_A->velocity.y = 0.0f;
    if (std::fabs(body_B->velocity.x) < VELOCITY_EPSILON)
        body_B->velocity.x = 0.0f;
    if (std::fabs(body_B->velocity.y) < VELOCITY_EPSILON)
        body_B->velocity.y = 0.0f;
}

// ====================================================================
// --- WORLD BOUNDARY (Boundary) ---
// ====================================================================

void collisionSystem::solve_boundary_contacts(world &simulation_world)
{
    for (body &current_body : simulation_world.bodies)
    {
        if (current_body.inv_mass == 0.0f)
            continue;

        const float ground_y_limit = 0.0f;

        if (current_body.position.y - current_body.radius < ground_y_limit)
        {
            // 1. Position correction
            current_body.position.y = ground_y_limit + current_body.radius;

            // 2. Velocity resolution (bounce)
            if (current_body.velocity.y < 0.0f)
            {
                float applied_restitution = current_body.restitution;
                current_body.velocity.y = -current_body.velocity.y * applied_restitution;
            }

            // 3. Low-velocity elimination
            if (std::fabs(current_body.velocity.y) < VELOCITY_EPSILON)
            {
                current_body.velocity.y = 0.0f;
            }
        }
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