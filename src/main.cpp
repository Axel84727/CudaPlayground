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

// World scale: 1 world unit (meter) = 10 pixels.
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
body create_body(float pos_x, float pos_y, float vel_x, float vel_y, float mass, float radius, float restitution, float damping = 0.0f, float friction = 0.0f)
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
    const vec2 gravity = vec2(0.0f, -9.8f);
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

    world sim_world(bodies, gravity, fixed_dt);

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

    // Initialize previous_position for the first Verlet step
    for (auto &b : sim_world.bodies)
    {
        // CRITICAL VERLET CORRECTION:
        // Set previous_position to reflect initial velocity (if any).
        // p_old = p_curr - v_init * dt
        b.previous_position = b.position - b.velocity * sim_world.delta_time;
    }

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
        for (size_t i = 0; i < sim_world.bodies.size(); ++i)
        {
            const auto &b = sim_world.bodies[i];
            float dx = b.position.x - click_world.x;
            float dy = b.position.y - click_world.y;
            float d2 = dx * dx + dy * dy;
            if (d2 < best_dist2 && d2 <= (b.radius * b.radius))
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
            snapshot = sim_world.bodies; // copy current state
        }
        if (IsKeyPressed(KEY_L))
        {
            if (!snapshot.empty())
            {
                sim_world.bodies = snapshot;
                // Recompute previous_position for all bodies to keep Verlet consistent
                for (auto &b : sim_world.bodies)
                {
                    b.inv_mass = (b.mass > 0.0f) ? 1.0f / b.mass : 0.0f;
                    float dt = sim_world.delta_time;
                    if (dt > 0.0f)
                        b.previous_position = b.position - b.velocity * dt;
                }
            }
        }

        // Run physics only when not paused, or single-step requested
        while (accumulator >= fixed_dt)
        {
            if (!paused || step_next)
            {
                manager.update(sim_world, fixed_dt); // Update physics
                step_next = false;
            }
            accumulator -= fixed_dt;
            if (paused)
                break; // when paused, only run one step per keypress
        }

        // boundary clamping removed: collisionSystem now handles wall/ground bounces

        // Smoothly move selected/dragged body towards mouse while dragging
        if (dragging && dragging_idx >= 0 && dragging_idx < (int)sim_world.bodies.size())
        {
            body &b = sim_world.bodies[dragging_idx];
            // latest mouse world
            vec2 latest_mouse = mouse_history[(mouse_history_idx - 1 + 8) % 8];
            // lerp factor (0..1) smaller = smoother
            float lerp_f = 0.25f;
            b.position = b.position * (1.0f - lerp_f) + latest_mouse * lerp_f;
            // zero velocity while dragging to avoid physics fighting the drag
            b.velocity = vec2(0, 0);
            float dt = sim_world.delta_time;
            if (dt > 0.0f)
                b.previous_position = b.position - b.velocity * dt;
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
            if (dragging && dragging_idx >= 0 && dragging_idx < (int)sim_world.bodies.size())
            {
                body &b = sim_world.bodies[dragging_idx];
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
                    b.velocity = mouse_vel * 0.5f;
                }
                else
                {
                    // fallback: small nudge
                    b.velocity = vec2(0, 0);
                }
                float dt = sim_world.delta_time;
                if (dt > 0.0f)
                    b.previous_position = b.position - b.velocity * dt;
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
            sim_world.bodies.push_back(create_body(wx, wy, 0.0f, 0.0f, spawn_mass, spawn_radius, spawn_restitution, spawn_damping, spawn_friction));
            auto &nb = sim_world.bodies.back();
            float dt = sim_world.delta_time;
            if (dt > 0.0f)
                nb.previous_position = nb.position - nb.velocity * dt;
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

        if (selected_body_index >= 0 && selected_body_index < (int)sim_world.bodies.size())
        {
            bool changed = false;
            body &sel = sim_world.bodies[selected_body_index];

            // Adjustments: M/B mass +/-, R/T restitution +/-, S/A radius +/-
            if (IsKeyPressed(KEY_M))
            {
                sel.mass += 0.1f;
                changed = true;
            }
            if (IsKeyPressed(KEY_B)) // alternative for lowercase b
            {
                sel.mass = std::max(0.0f, sel.mass - 0.1f);
                changed = true;
            }
            if (IsKeyPressed(KEY_R))
            {
                sel.restitution = std::min(1.0f, sel.restitution + 0.05f);
                changed = true;
            }
            if (IsKeyPressed(KEY_T)) // alternative for decreasing restitution
            {
                sel.restitution = std::max(0.0f, sel.restitution - 0.05f);
                changed = true;
            }
            if (IsKeyPressed(KEY_S))
            {
                sel.radius = sel.radius + 0.1f;
                changed = true;
            }
            if (IsKeyPressed(KEY_A)) // alternative for decreasing radius
            {
                sel.radius = std::max(0.1f, sel.radius - 0.1f);
                changed = true;
            }

            // Damping adjustments: Y increase, U decrease
            if (IsKeyPressed(KEY_Y))
            {
                sel.damping = std::max(0.0f, sel.damping - 0.01f);
                changed = true;
            }
            if (IsKeyPressed(KEY_U))
            {
                sel.damping += 0.01f;
                changed = true;
            }
            // Friction adjustments: G increase, H decrease
            if (IsKeyPressed(KEY_G))
            {
                sel.friction = std::max(0.0f, sel.friction - 0.01f);
                changed = true;
            }
            if (IsKeyPressed(KEY_H))
            {
                sel.friction += 0.01f;
                changed = true;
            }

            // Delete selected body (DEL or X)
            if (IsKeyPressed(KEY_X) || IsKeyPressed(KEY_DELETE))
            {
                sim_world.bodies.erase(sim_world.bodies.begin() + selected_body_index);
                selected_body_index = -1;
                continue; // skip further handling for this frame
            }

            // color controls removed

            if (changed)
            {
                // Recompute inv_mass and synchronize previous_position
                sel.inv_mass = (sel.mass > 0.0f) ? 1.0f / sel.mass : 0.0f;
                float dt = sim_world.delta_time;
                if (dt > 0.0f)
                {
                    sel.previous_position = sel.position - sel.velocity * dt;
                }
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
        for (size_t i = 0; i < sim_world.bodies.size(); ++i)
        {
            const auto &b = sim_world.bodies[i];
            vec2 screen_pos = WorldToScreen(b.position);
            int screen_radius = (int)(b.radius * world_scale);

            // Use fixed color per-body type: static=RED, dynamic=BLUE
            Color draw_color = (b.inv_mass == 0.0f) ? RED : BLUE;

            // Draw main circle
            DrawCircle((int)screen_pos.x, (int)screen_pos.y, screen_radius, draw_color);
            // Draw outline
            DrawCircleLines((int)screen_pos.x, (int)screen_pos.y, screen_radius, BLACK);

            // Label with id and mass above the body
            DrawText(TextFormat("#%d m:%.2f", (int)i, b.mass), (int)screen_pos.x - screen_radius, (int)screen_pos.y - screen_radius - 18, 12, WHITE);

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

        // 3. Draw FPS
        DrawFPS(10, 10);
        DrawText("Fixed DT: 1/60s", 10, 35, 20, WHITE);
        // Show pause/snapshot state
        // (replicating the paused variable by querying the key state here is not possible, we assume the key toggles)
        // Draw quick help
        DrawText("P: Pause/Resume  N: Step (when paused)  O: Save snapshot  L: Load snapshot", 10, 60, 14, LIGHTGRAY);

        // 4. Properties panel (if selected)
        if (selected_body_index >= 0 && selected_body_index < (int)sim_world.bodies.size())
        {
            const body &sel = sim_world.bodies[selected_body_index];
            int panel_x = screen_width - 260;
            int panel_y = 10;
            DrawRectangle(panel_x - 10, panel_y - 10, 250, 140, Fade(BLACK, 0.6f));
            DrawText(TextFormat("Selected: %d", selected_body_index), panel_x, panel_y, 18, YELLOW);
            DrawText(TextFormat("Mass: %.2f", sel.mass), panel_x, panel_y + 24, 16, WHITE);
            DrawText(TextFormat("InvMass: %.4f", sel.inv_mass), panel_x, panel_y + 44, 16, WHITE);
            DrawText(TextFormat("Radius: %.2f m", sel.radius), panel_x, panel_y + 64, 16, WHITE);
            DrawText(TextFormat("Restitution: %.2f", sel.restitution), panel_x, panel_y + 84, 16, WHITE);
            DrawText(TextFormat("Damping: %.2f", sel.damping), panel_x, panel_y + 104, 14, WHITE);
            DrawText(TextFormat("Friction: %.2f", sel.friction), panel_x, panel_y + 124, 14, WHITE);
            DrawText("Keys: M/B mass +/-, R/T restitution +/-, S/A radius +/-, Y/U damping, G/H friction", panel_x, panel_y + 144, 10, LIGHTGRAY);
        }

        // Show spawn params quick info
        DrawText(TextFormat("Spawn - mass:%.2f r:%.2f rest:%.2f damp:%.2f fric:%.2f (SPACE to spawn)", spawn_mass, spawn_radius, spawn_restitution, spawn_damping, spawn_friction), 10, 80, 12, LIGHTGRAY);
        // spawn color removed

        EndDrawing();
    }

    // --- 4. Resource cleanup ---
    CloseWindow();
    return 0;
}