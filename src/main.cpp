#include "raylib.h"
#include "physics/world.hpp"
#include "physics/body.hpp"
#include "math/vec2.hpp"
#include "sim/systemManager.hpp"
#include "sim/movementSystem.hpp"
#include "sim/collisionSystem.hpp"
#include <memory>
#include <iostream>
#include <vector>

// ====================================================================
// --- VISUALIZATION CONFIGURATION ---
// ====================================================================

// Window dimensions
const int screen_width = 1200;
const int screen_height = 800;

// World scale: 1 world unit (meter) = 10 pixels. (restored original visual scale)
const float world_scale = 10.0f;

// Window center points for coordinate mapping
const float center_x = screen_width / 2.0f;
const float center_y = screen_height / 2.0f;

// ====================================================================
// --- HELPER FUNCTIONS ---
// ====================================================================

/**
 * @brief Converts World coordinates (Y+ up) to Screen coordinates (Y+ down).
 * @param world_pos The position in world space.
 * @return vec2 with pixel coordinates.
 */
vec2 WorldToScreen(const vec2 &world_pos)
{
    // 1. Scale and center X: map world 0,0 to screen center X.
    float screen_x = world_pos.x * world_scale + center_x;

    // 2. Scale and invert Y: map world 0,0 to screen center Y, invert Y.
    float screen_y = center_y - world_pos.y * world_scale;

    return vec2(screen_x, screen_y);
}

/**
 * @brief Helper to create a body with inv_mass initialization.
 */
// Default parameters set per user request
body create_body(float pos_x, float pos_y, float vel_x, float vel_y, float mass = 1.60f, float radius = 1.80f, float restitution = 0.85f, float damping = 0.10f, float friction = 0.50f)
{
    float inv_mass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
    // Initial acceleration set to 0, since Verlet computes it from forces.
    return body(vec2(pos_x, pos_y), vec2(vel_x, vel_y), vec2(0, 0), mass, inv_mass, radius, restitution, damping, friction);
}

// ====================================================================
// --- MAIN LOOP ---
// ====================================================================

int main()
{
    // --- 1. Initialize raylib ---
    InitWindow(screen_width, screen_height, "Physics Engine (Verlet + raylib)");
    SetTargetFPS(144); // render FPS

    // --- 2. Simulation Initialization (Physical World) ---

    // Configuration
    // Global gravity set per user request
    const vec2 gravity = vec2(0.0f, -41.63f);
    const float fixed_dt = 1.0f / 60.0f; // fixed time step for physics (60Hz)

    // Initial bodies:
    std::vector<body> bodies;

    // Main falling ball (bounces)
    bodies.push_back(create_body(0.0f, 40.0f, 0.0f, 0.0f, 1.0f, 2.0f, 0.8f));

    // Elastic collision ball
    bodies.push_back(create_body(15.0f, 40.0f, -5.0f, 0.0f, 1.0f, 2.0f, 1.0f));

    // Inelastic collision ball (heading left)
    bodies.push_back(create_body(-15.0f, 40.0f, 5.0f, 0.0f, 1.0f, 2.0f, 0.5f));

    // Central static wall (inv_mass = 0)
    // bodies.push_back(create_body(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f));

    // CRITICAL VERLET CORRECTION:
    // Set previous_position in the initial body vector so add_body will initialize previous positions correctly.
    for (auto &b : bodies)
    {
        b.previous_position = b.position - b.velocity * fixed_dt;
    }

    // Create an empty SoA-first world and populate from initial bodies via add_body
    world sim_world;
    sim_world.gravity_x = gravity.x;
    sim_world.gravity_y = gravity.y;
    sim_world.delta_time = fixed_dt;
    for (auto &b : bodies)
        sim_world.add_body(b);

    // Debug: print initial SoA contents to help diagnose physics issues
    std::cout << "SIM_DEBUG: bodies=" << sim_world.size() << " dt=" << sim_world.delta_time << "\n";
    for (size_t i = 0; i < sim_world.size(); ++i)
    {
        std::cout << "SIM_DEBUG: B" << i << " pos=(" << sim_world.position_x[i] << "," << sim_world.position_y[i] << ") ";
        std::cout << "prev=(" << sim_world.previous_position_x[i] << "," << sim_world.previous_position_y[i] << ") ";
        std::cout << "vel=(" << sim_world.vel_x[i] << "," << sim_world.vel_y[i] << ") ";
        std::cout << "mass=" << ((i < sim_world.mass.size()) ? sim_world.mass[i] : 0.0f) << " inv_mass=" << ((i < sim_world.inv_mass.size()) ? sim_world.inv_mass[i] : 0.0f) << "\n";
    }

    // Adjust world grid bounds to match the visible window in world coordinates
    // So that wall/ceiling/ground collisions occur at the screen edges
    float vis_min_x = -center_x / world_scale;
    float vis_max_x = (screen_width - center_x) / world_scale;
    float vis_max_y = center_y / world_scale;
    float vis_min_y = -(screen_height - center_y) / world_scale;

    sim_world.grid_info.min_x = vis_min_x;
    sim_world.grid_info.max_x = vis_max_x;
    sim_world.grid_info.min_y = vis_min_y;
    sim_world.grid_info.max_y = vis_max_y;

    // Recompute grid sizes and resize grid storage (same logic as in world constructor)
    {
        float width = sim_world.grid_info.max_x - sim_world.grid_info.min_x;
        float height = sim_world.grid_info.max_y - sim_world.grid_info.min_y;
        int numCellsX = static_cast<int>(std::ceil(width / sim_world.grid_info.cell_size));
        int numCellsY = static_cast<int>(std::ceil(height / sim_world.grid_info.cell_size));
        sim_world.grid_info.num_cells_x = numCellsX;
        sim_world.grid_info.num_cells_y = numCellsY;
        int totalCells = std::max(1, numCellsX * numCellsY);
        sim_world.grid.clear();
        sim_world.grid.resize(totalCells);
    }

    // Note: previous_position was already initialized in the initial bodies vector before
    // constructing `sim_world` so the SoA previous_position arrays are correct.

    // Systems setup
    systemManager manager;
    manager.addSystem(std::make_unique<movementSystem>());
    manager.addSystem(std::make_unique<collisionSystem>());

    float accumulator = 0.0f;
    // --- Selection and on-screen UI ---
    int selected_body_index = -1;
    auto select_body_at_screen = [&](int mx, int my) -> int
    {
        // Convert screen to world
        float wx = (mx - center_x) / world_scale;
        float wy = (center_y - my) / world_scale;
        vec2 click_world(wx, wy);

        float best_dist2 = 1e30f;
        int best_idx = -1;
        size_t n = sim_world.size();
        for (size_t i = 0; i < n; ++i)
        {
            float px = sim_world.position_x[i];
            float py = sim_world.position_y[i];
            float r = (i < sim_world.radius.size()) ? sim_world.radius[i] : 0.0f;
            float dx = px - click_world.x;
            float dy = py - click_world.y;
            float d2 = dx * dx + dy * dy;
            if (d2 < best_dist2 && d2 <= (r * r))
            {
                best_dist2 = d2;
                best_idx = (int)i;
            }
        }
        return best_idx;
    };

    // --- Drag / Spawn state ---
    static bool dragging = false;
    static int dragging_idx = -1;
    // ring buffer of last mouse positions (world coords) to compute throw velocity
    static vec2 mouse_history[8];
    static int mouse_history_idx = 0;
    static int mouse_history_count = 0;
    vec2 last_mouse_world = vec2(0, 0);

    // Spawn parameters (modifiable with keys)
    static float spawn_mass = 1.0f;
    static float spawn_radius = 2.0f;
    static float spawn_restitution = 0.8f;
    static float spawn_damping = 0.0f;
    static float spawn_friction = 0.0f;
    // Runtime tuning: global damping and gravity scale
    static float gravity_scale = 1.0f;
    // Keys: '[' decrease damping, ']' increase damping
    //       ',' decrease gravity, '.' increase gravity
    // colors removed - rendering will use fixed colors (BLUE for dynamic, RED for static)

    // Other code continues...

    // --- 3. Main render and simulation loop ---
    while (!WindowShouldClose())
    {

        // --- A. Time Stepping (Stable physics with fixed step) ---
        accumulator += GetFrameTime();

        // --- Pause/step/snapshot controls ---
        static bool paused = false;
        static bool step_next = false;
        static std::vector<body> snapshot;

        if (IsKeyPressed(KEY_P))
        {
            paused = !paused;
        }
        if (IsKeyPressed(KEY_N))
        {
            // single step
            if (paused)
                step_next = true;
        }
        if (IsKeyPressed(KEY_O))
        {
            // copy current state by building an in-memory snapshot from SoA arrays
            snapshot.clear();
            size_t n = sim_world.size();
            snapshot.resize(n);
            for (size_t i = 0; i < n; ++i)
            {
                snapshot[i].position = sim_world.get_position(i);
                snapshot[i].previous_position = vec2(sim_world.previous_position_x[i], sim_world.previous_position_y[i]);
                snapshot[i].velocity = vec2(sim_world.vel_x[i], sim_world.vel_y[i]);
                snapshot[i].acceleration = vec2(sim_world.acc_x[i], sim_world.acc_y[i]);
                snapshot[i].mass = (i < sim_world.mass.size()) ? sim_world.mass[i] : 0.0f;
                snapshot[i].inv_mass = (i < sim_world.inv_mass.size()) ? sim_world.inv_mass[i] : 0.0f;
                snapshot[i].radius = (i < sim_world.radius.size()) ? sim_world.radius[i] : 0.0f;
                snapshot[i].damping = (i < sim_world.damping.size()) ? sim_world.damping[i] : 0.0f;
                snapshot[i].friction = (i < sim_world.friction.size()) ? sim_world.friction[i] : 0.0f;
                snapshot[i].restitution = (i < sim_world.restitution.size()) ? sim_world.restitution[i] : 1.0f;
            }
        }
        if (IsKeyPressed(KEY_L))
        {
            if (!snapshot.empty())
            {
                // prepare snapshot: recompute inv_mass and previous_position before restoring
                for (auto &b : snapshot)
                {
                    b.inv_mass = (b.mass > 0.0f) ? 1.0f / b.mass : 0.0f;
                    float dt = sim_world.delta_time;
                    if (dt > 0.0f)
                        b.previous_position = b.position - b.velocity * dt;
                }
                // Restore by clearing and re-adding bodies (simple approach)
                // Note: this keeps other per-world state (grid bounds)
                // but resets per-particle SoA arrays to snapshot values.
                sim_world.position_x.clear();
                sim_world.position_y.clear();
                sim_world.previous_position_x.clear();
                sim_world.previous_position_y.clear();
                sim_world.vel_x.clear();
                sim_world.vel_y.clear();
                sim_world.acc_x.clear();
                sim_world.acc_y.clear();
                sim_world.mass.clear();
                sim_world.inv_mass.clear();
                sim_world.radius.clear();
                sim_world.damping.clear();
                sim_world.friction.clear();
                sim_world.restitution.clear();
                for (auto &b : snapshot)
                {
                    sim_world.add_body(b);
                }
            }
        }

        // Run physics only when not paused, or single-step requested
        while (accumulator >= fixed_dt)
        {
            if (!paused || step_next)
            {
                // Apply runtime gravity scaling before the physics step
                sim_world.gravity_x = gravity.x * gravity_scale;
                sim_world.gravity_y = gravity.y * gravity_scale;
                manager.update(sim_world, fixed_dt); // Update physics
                static bool printed_after_step = false;
                if (!printed_after_step)
                {
                    std::cout << "SIM_DEBUG_POST: after first update" << "\n";
                    for (size_t i = 0; i < sim_world.size(); ++i)
                    {
                        std::cout << "SIM_DEBUG_POST: B" << i << " pos=(" << sim_world.position_x[i] << "," << sim_world.position_y[i] << ") ";
                        std::cout << "prev=(" << sim_world.previous_position_x[i] << "," << sim_world.previous_position_y[i] << ") ";
                        std::cout << "vel=(" << sim_world.vel_x[i] << "," << sim_world.vel_y[i] << ")\n";
                    }
                    printed_after_step = true;
                }
                step_next = false;
            }
            accumulator -= fixed_dt;
            if (paused)
                break; // when paused, only run one step per keypress
        }

        // boundary clamping removed: collisionSystem now handles wall/ground bounces

        // Smoothly move selected/dragged body towards mouse while dragging
        if (dragging && dragging_idx >= 0 && dragging_idx < (int)sim_world.size())
        {
            // Update position: set_position updates SoA arrays.
            // For velocity/previous_position we update the internal arrays directly.
            // Acquire new position below and assign via set_position.
            // latest mouse world
            vec2 latest_mouse = mouse_history[(mouse_history_idx - 1 + 8) % 8];
            // lerp factor (0..1) smaller = smoother
            float lerp_f = 0.25f;
            vec2 oldpos = sim_world.get_position(dragging_idx);
            vec2 newpos = oldpos * (1.0f - lerp_f) + latest_mouse * lerp_f;
            sim_world.set_position(dragging_idx, newpos);
            // zero velocity while dragging to avoid physics fighting the drag
            sim_world.vel_x[dragging_idx] = 0.0f;
            sim_world.vel_y[dragging_idx] = 0.0f;
            float dt = sim_world.delta_time;
            if (dt > 0.0f)
            {
                sim_world.previous_position_x[dragging_idx] = newpos.x - 0.0f * dt;
                sim_world.previous_position_y[dragging_idx] = newpos.y - 0.0f * dt;
            }
        }

        // --- INPUT: Drag / Spawn / Selection and property modification ---
        // Drag start (smooth)

        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON))
        {
            if (!dragging)
            {
                int mx = GetMouseX();
                int my = GetMouseY();
                int idx = select_body_at_screen(mx, my);
                if (idx >= 0)
                {
                    dragging = true;
                    dragging_idx = idx;
                }
            }
            int mx = GetMouseX();
            int my = GetMouseY();
            float wx = (mx - center_x) / world_scale;
            float wy = (center_y - my) / world_scale;
            // push into history
            mouse_history[mouse_history_idx] = vec2(wx, wy);
            mouse_history_idx = (mouse_history_idx + 1) % 8;
            mouse_history_count = std::min(mouse_history_count + 1, 8);
        }

        if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON))
        {
            if (dragging && dragging_idx >= 0 && dragging_idx < (int)sim_world.size())
            {
                // Update velocity on release. We update the SoA velocity and previous_position arrays
                // and keep internal SoA arrays updated.
                // compute average mouse velocity from history
                if (mouse_history_count >= 2)
                {
                    int oldest = (mouse_history_idx - mouse_history_count + 8) % 8;
                    vec2 oldest_pos = mouse_history[oldest];
                    vec2 newest_pos = mouse_history[(mouse_history_idx - 1 + 8) % 8];
                    vec2 delta = newest_pos - oldest_pos;
                    float dt_total = (float)mouse_history_count * (1.0f / 60.0f); // approximate frame dt
                    vec2 mouse_vel = (dt_total > 0.0f) ? delta * (1.0f / dt_total) : vec2(0, 0);
                    // apply a scaled down version as throw velocity
                    sim_world.vel_x[dragging_idx] = (mouse_vel * 0.5f).x;
                    sim_world.vel_y[dragging_idx] = (mouse_vel * 0.5f).y;
                }
                else
                {
                    // fallback: small nudge
                    sim_world.vel_x[dragging_idx] = 0.0f;
                    sim_world.vel_y[dragging_idx] = 0.0f;
                }
                float dt = sim_world.delta_time;
                if (dt > 0.0f)
                {
                    vec2 pos = sim_world.get_position(dragging_idx);
                    sim_world.previous_position_x[dragging_idx] = pos.x - sim_world.vel_x[dragging_idx] * dt;
                    sim_world.previous_position_y[dragging_idx] = pos.y - sim_world.vel_y[dragging_idx] * dt;
                }
            }
            dragging = false;
            dragging_idx = -1;
        }

        // Spawn new body with SPACE (at mouse)
        if (IsKeyPressed(KEY_SPACE))
        {
            int mx = GetMouseX();
            int my = GetMouseY();
            float wx = (mx - center_x) / world_scale;
            float wy = (center_y - my) / world_scale;
            body nb = create_body(wx, wy, 0.0f, 0.0f, spawn_mass, spawn_radius, spawn_restitution, spawn_damping, spawn_friction);
            float dt = sim_world.delta_time;
            if (dt > 0.0f)
                nb.previous_position = nb.position - nb.velocity * dt;
            sim_world.add_body(nb);
        }

        // Spawn parameter keys: 1/2 mass, 3/4 restitution, 5/6 radius
        if (IsKeyPressed(KEY_ONE))
            spawn_mass = std::max(0.01f, spawn_mass - 0.1f);
        if (IsKeyPressed(KEY_TWO))
            spawn_mass += 0.1f;
        if (IsKeyPressed(KEY_THREE))
            spawn_restitution = std::max(0.0f, spawn_restitution - 0.05f);
        if (IsKeyPressed(KEY_FOUR))
            spawn_restitution = std::min(1.0f, spawn_restitution + 0.05f);
        if (IsKeyPressed(KEY_FIVE))
            spawn_radius = std::max(0.1f, spawn_radius - 0.1f);
        if (IsKeyPressed(KEY_SIX))
            spawn_radius += 0.1f;
        // Tweak global damping
        if (IsKeyPressed(KEY_LEFT_BRACKET))
            sim_world.global_damping = std::max(0.0f, sim_world.global_damping - 0.005f);
        if (IsKeyPressed(KEY_RIGHT_BRACKET))
            sim_world.global_damping = std::min(0.5f, sim_world.global_damping + 0.005f);
        // Tweak gravity scale
        if (IsKeyPressed(KEY_COMMA))
            gravity_scale = std::max(0.0f, gravity_scale - 0.05f);
        if (IsKeyPressed(KEY_PERIOD))
            gravity_scale += 0.05f;
        // Spawn damping/friction keys: 7/8 damping -, + ; 9/0 friction -, +
        if (IsKeyPressed(KEY_SEVEN))
            spawn_damping = std::max(0.0f, spawn_damping - 0.05f);
        if (IsKeyPressed(KEY_EIGHT))
            spawn_damping += 0.05f;
        if (IsKeyPressed(KEY_NINE))
            spawn_friction = std::max(0.0f, spawn_friction - 0.05f);
        if (IsKeyPressed(KEY_ZERO))
            spawn_friction += 0.05f;
        // color controls removed

        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON))
        {
            int mx = GetMouseX();
            int my = GetMouseY();
            int idx = select_body_at_screen(mx, my);
            selected_body_index = idx;
        }

        if (selected_body_index >= 0 && selected_body_index < (int)sim_world.size())
        {
            bool changed = false;

            // Adjustments: M/B mass +/-, R/T restitution +/-, S/A radius +/-
            if (IsKeyPressed(KEY_M))
            {
                sim_world.mass[selected_body_index] += 0.1f;
                changed = true;
            }
            if (IsKeyPressed(KEY_B)) // alternative for lowercase b
            {
                sim_world.mass[selected_body_index] = std::max(0.0f, sim_world.mass[selected_body_index] - 0.1f);
                changed = true;
            }
            if (IsKeyPressed(KEY_R))
            {
                if (selected_body_index < sim_world.restitution.size())
                    sim_world.restitution[selected_body_index] = std::min(1.0f, sim_world.restitution[selected_body_index] + 0.05f);
                changed = true;
            }
            if (IsKeyPressed(KEY_T)) // alternative for decreasing restitution
            {
                if (selected_body_index < sim_world.restitution.size())
                    sim_world.restitution[selected_body_index] = std::max(0.0f, sim_world.restitution[selected_body_index] - 0.05f);
                changed = true;
            }
            if (IsKeyPressed(KEY_S))
            {
                sim_world.radius[selected_body_index] = sim_world.radius[selected_body_index] + 0.1f;
                changed = true;
            }
            if (IsKeyPressed(KEY_A)) // alternative for decreasing radius
            {
                sim_world.radius[selected_body_index] = std::max(0.1f, sim_world.radius[selected_body_index] - 0.1f);
                changed = true;
            }

            // Damping adjustments: Y increase, U decrease
            if (IsKeyPressed(KEY_Y))
            {
                if (selected_body_index < sim_world.damping.size())
                    sim_world.damping[selected_body_index] = std::max(0.0f, sim_world.damping[selected_body_index] - 0.01f);
                changed = true;
            }
            if (IsKeyPressed(KEY_U))
            {
                if (selected_body_index < sim_world.damping.size())
                    sim_world.damping[selected_body_index] += 0.01f;
                changed = true;
            }
            // Friction adjustments: G increase, H decrease
            if (IsKeyPressed(KEY_G))
            {
                if (selected_body_index < sim_world.friction.size())
                    sim_world.friction[selected_body_index] = std::max(0.0f, sim_world.friction[selected_body_index] - 0.01f);
                changed = true;
            }
            if (IsKeyPressed(KEY_H))
            {
                if (selected_body_index < sim_world.friction.size())
                    sim_world.friction[selected_body_index] += 0.01f;
                changed = true;
            }

            // Delete selected body (DEL or X)
            if (IsKeyPressed(KEY_X) || IsKeyPressed(KEY_DELETE))
            {
                sim_world.remove_body(selected_body_index);
                selected_body_index = -1;
                continue; // skip further handling for this frame
            }

            if (changed)
            {
                // Recompute inv_mass and synchronize previous_position in SoA arrays
                float massVal = sim_world.mass[selected_body_index];
                sim_world.inv_mass[selected_body_index] = (massVal > 0.0f) ? 1.0f / massVal : 0.0f;
                float dt = sim_world.delta_time;
                if (dt > 0.0f)
                {
                    float vx = sim_world.vel_x[selected_body_index];
                    float vy = sim_world.vel_y[selected_body_index];
                    sim_world.previous_position_x[selected_body_index] = sim_world.position_x[selected_body_index] - vx * dt;
                    sim_world.previous_position_y[selected_body_index] = sim_world.position_y[selected_body_index] - vy * dt;
                }
                // SoA arrays are canonical. If UI changed SoA arrays above, they are ready.
            }
        }

        // --- B. Rendering (Visualization) ---
        BeginDrawing();
        ClearBackground(DARKGRAY);

        // 1. Draw the ground (Ground_Y_Limit = 0.0f in the simulation)
        vec2 ground_screen_pos = WorldToScreen(vec2(0.0f, 0.0f));
        // Draw a white line at world Y = 0.0f
        DrawLine(0, (int)ground_screen_pos.y, screen_width, (int)ground_screen_pos.y, WHITE);
        DrawText("Ground (Y = 0.0m)", 10, (int)ground_screen_pos.y - 20, 20, WHITE);

        // 2. Draw bodies (and labels)
        for (size_t i = 0; i < sim_world.size(); ++i)
        {
            vec2 pos(sim_world.position_x[i], sim_world.position_y[i]);
            int screen_radius = (int)(sim_world.radius[i] * world_scale);
            vec2 screen_pos = WorldToScreen(pos);

            // Use fixed color per-body type: static=RED, dynamic=BLUE
            Color draw_color = (sim_world.inv_mass[i] == 0.0f) ? RED : BLUE;

            // Draw main circle
            DrawCircle((int)screen_pos.x, (int)screen_pos.y, screen_radius, draw_color);
            // Draw outline
            DrawCircleLines((int)screen_pos.x, (int)screen_pos.y, screen_radius, BLACK);

            // Label with id and mass above the body
            float mass = (i < sim_world.mass.size()) ? sim_world.mass[i] : 0.0f;
            DrawText(TextFormat("#%d m:%.2f", (int)i, mass), (int)screen_pos.x - screen_radius, (int)screen_pos.y - screen_radius - 18, 12, WHITE);

            // Highlight if selected
            if ((int)i == selected_body_index)
            {
                DrawCircleLines((int)screen_pos.x, (int)screen_pos.y, screen_radius + 4, YELLOW);
                // Mark with small label
                DrawText("SELECTED", (int)screen_pos.x - screen_radius, (int)screen_pos.y + screen_radius + 6, 12, YELLOW);
            }
        }

        // If none selected, show brief help
        if (selected_body_index < 0)
        {
            DrawText("Click a body to select it. Keys: M/B mass +/-, R/T restitution +/-, S/A radius +/-", 10, screen_height - 24, 14, LIGHTGRAY);
        }

        // 3. Draw stacked HUD (avoid overlaps)
        const int hud_x = 10;
        int hud_y = 10;
        const int hud_line_h = 20;
        // Primary HUD lines
        DrawFPS(hud_x, hud_y);
        hud_y += hud_line_h;
        DrawText("Fixed DT: 1/60s", hud_x, hud_y, 16, WHITE);
        hud_y += hud_line_h;
        DrawText(TextFormat("Gravity: %.2fm/s^2 (use , . to +/-)", sim_world.gravity_y * gravity_scale), hud_x, hud_y, 16, WHITE);
        hud_y += hud_line_h;
        DrawText(TextFormat("Global damping: %.4f (use [ ] to +/-)", sim_world.global_damping), hud_x, hud_y, 16, WHITE);
        hud_y += hud_line_h;
        DrawText("P: Pause/Resume  N: Step (when paused)", hud_x, hud_y, 14, LIGHTGRAY);
        hud_y += hud_line_h;
        DrawText("O: Save snapshot  L: Load snapshot  SPACE: Spawn", hud_x, hud_y, 14, LIGHTGRAY);
        hud_y += hud_line_h;

        // 4. Properties panel (if selected)
        if (selected_body_index >= 0 && selected_body_index < (int)sim_world.size())
        {
            // Read properties from SoA arrays
            float sel_mass = (selected_body_index < sim_world.mass.size()) ? sim_world.mass[selected_body_index] : 0.0f;
            float sel_inv_mass = (selected_body_index < sim_world.inv_mass.size()) ? sim_world.inv_mass[selected_body_index] : 0.0f;
            float sel_radius = (selected_body_index < sim_world.radius.size()) ? sim_world.radius[selected_body_index] : 0.0f;
            // Prefer SoA values.
            float sel_restitution = sim_world.get_restitution(selected_body_index);
            float sel_damping = sim_world.get_damping(selected_body_index);
            float sel_friction = sim_world.get_friction(selected_body_index);
            int panel_x = screen_width - 260;
            int panel_y = 10;
            DrawRectangle(panel_x - 10, panel_y - 10, 250, 140, Fade(BLACK, 0.6f));
            DrawText(TextFormat("Selected: %d", selected_body_index), panel_x, panel_y, 18, YELLOW);
            DrawText(TextFormat("Mass: %.2f", sel_mass), panel_x, panel_y + 24, 16, WHITE);
            DrawText(TextFormat("InvMass: %.4f", sel_inv_mass), panel_x, panel_y + 44, 16, WHITE);
            DrawText(TextFormat("Radius: %.2f m", sel_radius), panel_x, panel_y + 64, 16, WHITE);
            DrawText(TextFormat("Restitution: %.2f", sel_restitution), panel_x, panel_y + 84, 16, WHITE);
            DrawText(TextFormat("Damping: %.2f", sel_damping), panel_x, panel_y + 104, 14, WHITE);
            DrawText(TextFormat("Friction: %.2f", sel_friction), panel_x, panel_y + 124, 14, WHITE);
            DrawText("Keys: M/B mass +/-, R/T restitution +/-, S/A radius +/-, Y/U damping, G/H friction", panel_x, panel_y + 144, 10, LIGHTGRAY);
        }

        // Show spawn params quick info (moved down to avoid overlap with HUD)
        int spawn_info_y = hud_y + 6;
        DrawText(TextFormat("Spawn - mass:%.2f r:%.2f rest:%.2f damp:%.2f fric:%.2f (SPACE to spawn)", spawn_mass, spawn_radius, spawn_restitution, spawn_damping, spawn_friction), 10, spawn_info_y, 12, LIGHTGRAY);
        // spawn color removed

        EndDrawing();
    }

    // --- 4. Resource cleanup ---
    CloseWindow();
    return 0;
}