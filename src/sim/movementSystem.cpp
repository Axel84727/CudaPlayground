#include "sim/movementSystem.hpp"
#include "physics/body.hpp"
#include "physics/world.hpp"
movementSystem::movementSystem() {}
movementSystem::~movementSystem() {}
void movementSystem::verlet_integration(world &simulation_world)
{
    // Pre-calculate time terms for efficiency and clarity
    const float delta_time = simulation_world.delta_time;
    const float delta_time_squared = delta_time * delta_time;
    const float inverse_delta_time = 1.0f / delta_time;

    // Iterate over all bodies in the world
    for (body &current_body : simulation_world.bodies)
    {
        // 1. Check Static Bodies (inv_mass = 0)
        if (current_body.inv_mass <= 0.0f)
        {
            continue;
        }

        // 2. Calculation of Total Acceleration
        // Start with global gravity
        vec2 total_acceleration = simulation_world.gravity_vector;

        // Apply per-body viscous damping (proportional to velocity): a_damping = -damping * v
        if (current_body.damping != 0.0f)
        {
            total_acceleration = total_acceleration - (current_body.velocity * current_body.damping);
        }

        // Apply simple friction-like damping (approx Coulomb): a_friction = -friction * normalize(v) * |v|
        if (current_body.friction != 0.0f)
        {
            float speed = std::sqrt(current_body.velocity.x * current_body.velocity.x + current_body.velocity.y * current_body.velocity.y);
            if (speed > 1e-6f)
            {
                vec2 vel_dir = current_body.velocity * (1.0f / speed);
                total_acceleration = total_acceleration - (vel_dir * (current_body.friction * speed));
            }
        }

        // Save the current position before modifying it (will become the previous position)
        vec2 current_position = current_body.position;

        // 3. Calculation of the Next Position (Verlet equation)

        // Acceleration term: a * delta_time^2
        vec2 acceleration_term = total_acceleration * delta_time_squared;

        // Core Verlet equation:
        // next_pos = 2 * current_pos - previous_pos + a * delta_time^2
        vec2 next_position = (current_body.position * 2.0f) - current_body.previous_position + acceleration_term;

        // 4. Update Position for the Next Step
        current_body.previous_position = current_position; // The old position is the current position
        current_body.position = next_position;

        // 5. Explicit Velocity Calculation (Needed for collision detection)
        // velocity = (current_pos - previous_pos) / delta_time
        current_body.velocity = (current_body.position - current_body.previous_position) * inverse_delta_time;
    }
}

void movementSystem::update(world &simulation_world, float delta_time)
{
    verlet_integration(simulation_world);
}