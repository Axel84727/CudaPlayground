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
// --- CONFIGURACIÓN DE VISUALIZACIÓN ---
// ====================================================================

// Dimensiones de la Ventana
const int screen_width = 1200;
const int screen_height = 800;

// Escala del Mundo: 1 unidad de mundo (metro) = 10 píxeles.
const float world_scale = 10.0f;

// Puntos centrales de la ventana para mapeo de coordenadas
const float center_x = screen_width / 2.0f;
const float center_y = screen_height / 2.0f;

// ====================================================================
// --- FUNCIONES AUXILIARES ---
// ====================================================================

/**
 * @brief Convierte coordenadas de Mundo (Y+ arriba) a coordenadas de Pantalla (Y+ abajo).
 * @param world_pos La posición en el mundo físico.
 * @return vec2 con las coordenadas en píxeles.
 */
vec2 WorldToScreen(const vec2 &world_pos)
{
    // 1. Escalado y Centrado X: Mapea 0,0 del mundo al centro X de la pantalla.
    float screen_x = world_pos.x * world_scale + center_x;

    // 2. Escalado e Inversión Y: Mapea 0,0 del mundo al centro Y, e invierte Y.
    float screen_y = center_y - world_pos.y * world_scale;

    return vec2(screen_x, screen_y);
}

/**
 * @brief Función auxiliar para crear un cuerpo con inicialización de inv_mass.
 */
body create_body(float pos_x, float pos_y, float vel_x, float vel_y, float mass, float radius, float restitution)
{
    float inv_mass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
    // La aceleración inicial se pone en 0, ya que Verlet la calcula a partir de la fuerza.
    return body(vec2(pos_x, pos_y), vec2(vel_x, vel_y), vec2(0, 0), mass, inv_mass, radius, restitution);
}

// ====================================================================
// --- BUCLE PRINCIPAL ---
// ====================================================================

int main()
{
    // --- 1. Inicialización de raylib ---
    InitWindow(screen_width, screen_height, "Physics Engine (Verlet + raylib)");
    SetTargetFPS(144); // FPS de renderizado

    // --- 2. Inicialización de la Simulación (Mundo Físico) ---

    // Configuración
    const vec2 gravity = vec2(0.0f, -9.8f);
    const float fixed_dt = 1.0f / 60.0f; // Paso de tiempo fijo para la física (60Hz)

    // Cuerpos Iniciales:
    std::vector<body> bodies;

    // Bola principal cayendo (rebota)
    bodies.push_back(create_body(0.0f, 40.0f, 0.0f, 0.0f, 1.0f, 2.0f, 0.8f));

    // Bola de colisión elástica
    bodies.push_back(create_body(15.0f, 40.0f, -5.0f, 0.0f, 1.0f, 2.0f, 1.0f));

    // Bola de colisión inelástica (va hacia la izquierda)
    bodies.push_back(create_body(-15.0f, 40.0f, 5.0f, 0.0f, 1.0f, 2.0f, 0.5f));

    // Muro estático central (inv_mass = 0)
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

    // Inicialización de previous_position para la primera ejecución de Verlet
    for (auto &b : sim_world.bodies)
    {
        // CORRECCIN CR	TICA DE VERLET:
        // Establecer previous_position para reflejar la velocidad inicial (si la hay).
        // p_old = p_curr - v_init * dt
        b.previous_position = b.position - b.velocity * sim_world.delta_time;
    }

    // Configuración de Sistemas
    systemManager manager;
    manager.addSystem(std::make_unique<movementSystem>());
    manager.addSystem(std::make_unique<collisionSystem>());

    float accumulator = 0.0f;
    // --- Selección y UI en pantalla ---
    int selected_body_index = -1;
    auto select_body_at_screen = [&](int mx, int my) -> int
    {
        // Convierte pantalla a mundo
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

    // --- 3. Bucle Principal de Renderizado y Simulación ---
    while (!WindowShouldClose())
    {

        // --- A. Time Stepping (Física estable con paso fijo) ---
        accumulator += GetFrameTime();

        // --- Controles de pausa/step/snapshot ---
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
                manager.update(sim_world, fixed_dt); // Actualiza la física
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

        // --- INPUT: Drag / Spawn / Selección y modificación de propiedades ---
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
        static float spawn_mass = 1.0f;
        static float spawn_radius = 2.0f;
        static float spawn_restitution = 0.8f;

        if (IsKeyPressed(KEY_SPACE))
        {
            int mx = GetMouseX();
            int my = GetMouseY();
            float wx = (mx - center_x) / world_scale;
            float wy = (center_y - my) / world_scale;
            sim_world.bodies.push_back(create_body(wx, wy, 0.0f, 0.0f, spawn_mass, spawn_radius, spawn_restitution));
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

            // Ajustes: M/m = masa +/-, R/r = restitution +/-, S/s = radius +/-
            if (IsKeyPressed(KEY_M))
            {
                sel.mass += 0.1f;
                changed = true;
            }
            if (IsKeyPressed(KEY_B)) // pequeña masa abajo (usamos B como minúscula alternativa)
            {
                sel.mass = std::max(0.0f, sel.mass - 0.1f);
                changed = true;
            }
            if (IsKeyPressed(KEY_R))
            {
                sel.restitution = std::min(1.0f, sel.restitution + 0.05f);
                changed = true;
            }
            if (IsKeyPressed(KEY_T)) // R minúscula alternativa
            {
                sel.restitution = std::max(0.0f, sel.restitution - 0.05f);
                changed = true;
            }
            if (IsKeyPressed(KEY_S))
            {
                sel.radius = sel.radius + 0.1f;
                changed = true;
            }
            if (IsKeyPressed(KEY_A)) // s minúscula alternativa
            {
                sel.radius = std::max(0.1f, sel.radius - 0.1f);
                changed = true;
            }

            if (changed)
            {
                // Recalcular inv_mass y sincronizar previous_position
                sel.inv_mass = (sel.mass > 0.0f) ? 1.0f / sel.mass : 0.0f;
                float dt = sim_world.delta_time;
                if (dt > 0.0f)
                {
                    sel.previous_position = sel.position - sel.velocity * dt;
                }
            }
        }

        // --- B. Renderizado (Visualización) ---
        BeginDrawing();
        ClearBackground(DARKGRAY);

        // 1. Dibuja el suelo (Ground_Y_Limit = 0.0f en la simulación)
        vec2 ground_screen_pos = WorldToScreen(vec2(0.0f, 0.0f));
        // Dibujamos una línea blanca en la posición 0.0f del mundo
        DrawLine(0, (int)ground_screen_pos.y, screen_width, (int)ground_screen_pos.y, WHITE);
        DrawText("Ground (Y = 0.0m)", 10, (int)ground_screen_pos.y - 20, 20, WHITE);

        // 2. Dibuja los cuerpos (y etiquetas)
        for (size_t i = 0; i < sim_world.bodies.size(); ++i)
        {
            const auto &b = sim_world.bodies[i];
            vec2 screen_pos = WorldToScreen(b.position);
            int screen_radius = (int)(b.radius * world_scale);

            // Determinar color (Rojo si es estático, Azul si es dinámico)
            Color color = (b.inv_mass == 0.0f) ? RED : BLUE;

            // Dibuja el círculo principal
            DrawCircle((int)screen_pos.x, (int)screen_pos.y, screen_radius, color);
            // Dibuja el contorno
            DrawCircleLines((int)screen_pos.x, (int)screen_pos.y, screen_radius, BLACK);

            // Etiqueta con id y masa encima del cuerpo
            DrawText(TextFormat("#%d m:%.2f", (int)i, b.mass), (int)screen_pos.x - screen_radius, (int)screen_pos.y - screen_radius - 18, 12, WHITE);

            // Resaltar si es el seleccionado
            if ((int)i == selected_body_index)
            {
                DrawCircleLines((int)screen_pos.x, (int)screen_pos.y, screen_radius + 4, YELLOW);
                // Marca con flecha o rectángulo pequeño
                DrawText("SELECTED", (int)screen_pos.x - screen_radius, (int)screen_pos.y + screen_radius + 6, 12, YELLOW);
            }
        }

        // Si no hay seleccionado, mostrar help breve
        if (selected_body_index < 0)
        {
            DrawText("Click a body to select it. Keys: M/B mass +/-, R/T restitution +/-, S/A radius +/-", 10, screen_height - 24, 14, LIGHTGRAY);
        }

        // 3. Dibuja el FPS
        DrawFPS(10, 10);
        DrawText("Fixed DT: 1/60s", 10, 35, 20, WHITE);
        // Mostrar estado de pausa/snapshot
        // (replicar la variable paused consultando el estado de la tecla no es posible aquí, asumimos tecla toggles)
        // Dibujar ayuda rápida
        DrawText("P: Pause/Resume  N: Step (when paused)  O: Save snapshot  L: Load snapshot", 10, 60, 14, LIGHTGRAY);

        // 4. Panel de propiedades (si hay seleccionado)
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
            DrawText("Keys: M/B mass +/-, R/T restitution +/-, S/A radius +/-", panel_x, panel_y + 110, 12, LIGHTGRAY);
        }

        EndDrawing();
    }

    // --- 4. Liberación de Recursos ---
    CloseWindow();
    return 0;
}