#pragma once

#include <vector>
#include <utility>
#include "sim/ISystem.hpp"
#include "math/vec2.hpp"

class body;
class world;

// ====================================================================
// --- COLLISION DATA STRUCTURES ---
// The ContactManifold structure stores all key collision information
// so that resolve_collision can be cleaner.
// ====================================================================

struct ContactManifold
{
    body *body_A;                // Pointer to the first body involved
    body *body_B;                // Pointer to the second body involved
    vec2 normal_direction;       // Unit vector of the collision (direction from A to B)
    float penetration_depth;     // Magnitude of the overlap between bodies
    float effective_restitution; // Average coefficient of restitution
    float inverse_mass_sum;      // Sum of inverse masses (1/mA + 1/mB)
};

class collisionSystem : public ISystem
{
private:
    // --- SPATIAL GRID PHASES (Spatial Hashing) ---
    void clear_spatial_grid(world &simulation_world);
    void populate_spatial_grid(world &simulation_world);

    // --- COLLISION DETECTION PHASES ---
    // Broad Phase: Generates a list of pairs of nearby bodies (candidates).
    std::vector<std::pair<body *, body *>> broad_phase_generate_pairs(world &simulation_world);

    // Narrow Phase: Iterates over candidate pairs to check and resolve exact collisions.
    void narrow_phase_check_and_resolve(world &simulation_world);

    // Circle-Circle Check: Uses squared distances for efficiency.
    bool check_for_overlap(body *body_A, body *body_B, world &simulation_world);

    // Resolution: Applies positional correction and velocity impulse.
    void resolve_contact_with_impulse(body *body_A, body *body_B, world &simulation_world);

    // World Boundary Collisions (floor, walls).
    void solve_boundary_contacts(world &simulation_world);

public:
    // Main update loop of the collision simulation.
    void update(world &simulation_world, float delta_time) override;

    collisionSystem();
    ~collisionSystem();
};