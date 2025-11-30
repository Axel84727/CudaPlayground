#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include <cmath>   // Para std::isnan, std::isinf
#include <iomanip> // Para std::setprecision
#include "world.hpp"

// ================================================================
// DECLARACIONES DE FUNCIONES DE PRUEBA
// ================================================================

// M1: Constructores
void test_constructors();
// M2/M3: Inicialización y Simulación simple
void test_random_initialization();
// M4: Colisiones
void test_elastic_collision();
void test_static_collision();
// M5: Integradores
void test_integrator_stability(IntegratorType type, const char *name);

// ---------------------------------------------------------------
// Función auxiliar para imprimir vec2 (Mantenerla en global)
// ---------------------------------------------------------------
void print_vec2(const vec2 &v)
{
    std::cout << "(" << v.x << ", " << v.y << ")";
}

// ================================================================
// IMPLEMENTACIONES DE FUNCIONES DE PRUEBA
// ================================================================

// ---------------------------------------------------------------
// Test 1: Pruebas de Constructores (M1)
// ---------------------------------------------------------------
void test_constructors()
{
    std::cout << "--- PRUEBA DE CONSTRUCTORES (M1) ---" << std::endl;

    // 1. Pruebas de vec2
    vec2 v_cero;
    std::cout << "1.1 Vec2 por Defecto: ";
    print_vec2(v_cero);
    std::cout << std::endl;

    vec2 v_pos_b1(10.0f, 5.0f);
    vec2 v_vel_b1(2.0f, 0.0f);
    vec2 v_accel_b1(0.0f, -9.8f);

    std::cout << "1.2 Vec2 Parametrizado: ";
    print_vec2(v_pos_b1);
    std::cout << std::endl;

    // 2. Pruebas de body
    float masa_b1 = 50.0f;
    float inv_masa_b1 = 1.0f / masa_b1;

    body b1(v_pos_b1, v_vel_b1, v_accel_b1, masa_b1, inv_masa_b1, 1.0f);

    std::cout << "2. Body Inicializado:" << std::endl;
    std::cout << "   Posicion: ";
    print_vec2(b1.posicion);
    std::cout << ", Masa: " << b1.masa << ", Inv_M: " << b1.inv_mass << std::endl;

    // 3. Pruebas de world (Sobrecarga de Constructores)
    world mundo_vacio;
    std::cout << "3.1 World por Defecto: Bodies = " << mundo_vacio.bodies.size();
    std::cout << ", Gravedad = ";
    print_vec2(mundo_vacio.gravedad);
    std::cout << std::endl;

    std::vector<body> cuerpos_iniciales;
    cuerpos_iniciales.push_back(b1);
    body b2(vec2(-10.0f, 0.0f), v_vel_b1, v_accel_b1, 5.0f, 1.0f / 5.0f, 0.3f);
    cuerpos_iniciales.push_back(b2);

    vec2 g_simulacion(0.0f, -10.0f);
    world mi_mundo(cuerpos_iniciales, g_simulacion, 0.016f);

    std::cout << "3.2 World Parametrizado:" << std::endl;
    std::cout << "   Total Bodies: " << mi_mundo.bodies.size() << std::endl;
    std::cout << "   Gravedad: ";
    print_vec2(mi_mundo.gravedad);
    std::cout << ", Delta Time: " << mi_mundo.delta_time << std::endl;
    std::cout << "   Posicion del Body 1 copiado: ";
    print_vec2(mi_mundo.bodies[0].posicion);
    std::cout << "\n--------------------------------------------------------\n";
}

// ---------------------------------------------------------------
// Test 2: Prueba de Inicialización Aleatoria y Simulación Simple (M2/M3)
// ---------------------------------------------------------------
void test_random_initialization()
{
    std::cout << "--- PRUEBA DE INICIALIZACIÓN Y SIMULACIÓN SIMPLE (M2/M3) ---\n";

    std::vector<body> cuerpos;
    constexpr int NUM_ENTIDADES = 100;
    constexpr float RANGO_POS = 50.0f;
    constexpr float MIN_MASA = 1.0f;
    constexpr float MAX_MASA = 10.0f;

    cuerpos.reserve(NUM_ENTIDADES);

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 generador(seed);

    std::uniform_real_distribution<float> dist_pos(-RANGO_POS, RANGO_POS);
    std::uniform_real_distribution<float> dist_vel(-5.0f, 5.0f);
    std::uniform_real_distribution<float> dist_masa(MIN_MASA, MAX_MASA);
    std::uniform_real_distribution<float> dist_radio(0.5f, 2.0f);

    for (int i = 0; i < NUM_ENTIDADES; i++)
    {
        float masa = dist_masa(generador);
        float inv_masa = (masa > 0.0f) ? 1.0f / masa : 0.0f;

        body new_body(
            vec2(dist_pos(generador), dist_pos(generador)),
            vec2(dist_vel(generador), dist_vel(generador)),
            vec2(0.0f, 0.0f),
            masa,
            inv_masa,
            dist_radio(generador));

        cuerpos.push_back(new_body);
    }

    vec2 gravedad_sim(0.0f, -9.81f);
    float dt = 1.0f / 60.0f;

    world world1(cuerpos, gravedad_sim, dt);

    std::cout << "----INICIALIZACION EXITOSA----\n";
    std::cout << "World inicializado con " << world1.bodies.size() << " cuerpos.\n";
    std::cout << "El primer cuerpo esta en x:" << world1.bodies[0].posicion.x << " y: " << world1.bodies[0].posicion.y << "\n";

    std::cout << "\n--- EJECUTANDO BUCLE DE SIMULACION ---\n";

    int max_ticks = 100;
    for (int t = 0; t < max_ticks; ++t)
    {
        world1.update();
        if (t % 10 == 0)
        {
            std::cout << "Tick " << t
                      << ": Pos Y -> " << std::fixed << std::setprecision(4) << world1.bodies[0].posicion.y
                      << " | Vel Y -> " << std::fixed << std::setprecision(4) << world1.bodies[0].velocidad.y << "\n";
        }
    }
    std::cout << "\n--- RESULTADO FINAL ---\n";
    std::cout << "Cuerpo 0: Posicion Final Y: " << std::fixed << std::setprecision(4) << world1.bodies[0].posicion.y << "\n";
    std::cout << "--------------------------------------------------------\n";
}

// ---------------------------------------------------------------
// Test 3: Colisión frontal elástica (M4)
// ---------------------------------------------------------------
void test_elastic_collision()
{
    std::cout << "========================================================\n";
    std::cout << "PRUEBA 3: COLISION FRONTAL ELASTICA (M4)\n";
    std::cout << "OBJETIVO: Verificar separacion y cambio de velocidad (rebote).\n";
    std::cout << "========================================================\n";

    const float dt = 1.0f / 60.0f;
    const float radio = 1.0f;
    const float masa = 1.0f;
    const float inv_masa = 1.0f;
    const float vel_inicial = 5.0f;
    const float distancia_inicial = 2.0f;

    body A(vec2(-distancia_inicial / 2.0f, 0.0f), vec2(vel_inicial, 0.0f), vec2(0.0f, 0.0f), masa, inv_masa, radio);
    body B(vec2(distancia_inicial / 2.0f, 0.0f), vec2(-vel_inicial, 0.0f), vec2(0.0f, 0.0f), masa, inv_masa, radio);

    std::vector<body> bodies = {A, B};
    world collision_world(bodies, vec2(0.0f, 0.0f), dt);

    std::cout << "ESTADO INICIAL:\n";
    std::cout << "Body A | Pos X: " << std::fixed << std::setprecision(4) << collision_world.bodies[0].posicion.x << ", Vel X: " << collision_world.bodies[0].velocidad.x << std::endl;
    std::cout << "Body B | Pos X: " << std::fixed << std::setprecision(4) << collision_world.bodies[1].posicion.x << ", Vel X: " << collision_world.bodies[1].velocidad.x << std::endl;

    std::cout << "\n--- EJECUTANDO TICK 1 (COLISION Y RESOLUCION) ---\n";
    collision_world.update();

    float expected_vel_after = -(0.8f * vel_inicial);

    std::cout << "ESTADO FINAL:\n";
    std::cout << "Body A | Pos X: " << std::fixed << std::setprecision(4) << collision_world.bodies[0].posicion.x << ", Vel X: " << std::fixed << std::setprecision(4) << collision_world.bodies[0].velocidad.x << std::endl;
    std::cout << "Body B | Pos X: " << std::fixed << std::setprecision(4) << collision_world.bodies[1].posicion.x << ", Vel X: " << std::fixed << std::setprecision(4) << collision_world.bodies[1].velocidad.x << std::endl;

    if (std::abs(collision_world.bodies[0].velocidad.x - expected_vel_after) < 0.2f)
    {
        std::cout << "TEST RESULTADO: COLISIÓN FRONTAL - EXITOSA (Rebote OK).\n";
    }
    else
    {
        std::cout << "TEST RESULTADO: COLISIÓN FRONTAL - FALLIDA.\n";
    }
    std::cout << "\n";
}

// ---------------------------------------------------------------
// Test 4: Colisión con cuerpo estático (M4)
// ---------------------------------------------------------------
void test_static_collision()
{
    std::cout << "========================================================\n";
    std::cout << "PRUEBA 4: COLISION CON CUERPO ESTATICO (M4)\n";
    std::cout << "OBJETIVO: Verificar que el estático no se mueve y el dinámico rebota.\n";
    std::cout << "========================================================\n";

    const float dt = 1.0f / 60.0f;
    const float radio = 1.0f;
    const float masa_movil = 1.0f;
    const float vel_inicial = 5.0f;

    body A(vec2(-2.0f, 0.0f), vec2(vel_inicial, 0.0f), vec2(0.0f, 0.0f), masa_movil, 1.0f / masa_movil, radio);
    body B(vec2(0.0f, 0.0f), vec2(0.0f, 0.0f), vec2(0.0f, 0.0f), 0.0f, 0.0f, radio);

    std::vector<body> bodies = {A, B};
    world collision_world(bodies, vec2(0.0f, 0.0f), dt);

    std::cout << "ESTADO INICIAL:\n";
    std::cout << "Body A (Movil) | Pos X: " << std::fixed << std::setprecision(4) << collision_world.bodies[0].posicion.x << ", Vel X: " << collision_world.bodies[0].velocidad.x << std::endl;
    std::cout << "Body B (Estatico) | Pos X: " << std::fixed << std::setprecision(4) << collision_world.bodies[1].posicion.x << ", Vel X: " << collision_world.bodies[1].velocidad.x << std::endl;

    std::cout << "\n--- EJECUTANDO TICK 1 (COLISION CON PARED) ---\n";
    collision_world.update();

    float expected_vel_A = -(0.8f * vel_inicial);

    std::cout << "ESTADO FINAL:\n";
    std::cout << "Body A (Movil) | Pos X: " << std::fixed << std::setprecision(4) << collision_world.bodies[0].posicion.x << ", Vel X: " << std::fixed << std::setprecision(4) << collision_world.bodies[0].velocidad.x << std::endl;
    std::cout << "Body B (Estatico) | Pos X: " << std::fixed << std::setprecision(4) << collision_world.bodies[1].posicion.x << ", Vel X: " << std::fixed << std::setprecision(4) << collision_world.bodies[1].velocidad.x << std::endl;

    if (collision_world.bodies[1].posicion.x == 0.0f && collision_world.bodies[1].velocidad.x == 0.0f && std::abs(collision_world.bodies[0].velocidad.x - expected_vel_A) < 0.2f)
    {
        std::cout << "TEST RESULTADO: ESTATICO - EXITOSA (Estático no se movió; Movil rebotó OK).\n";
    }
    else
    {
        std::cout << "TEST RESULTADO: ESTATICO - FALLIDA.\n";
    }
    std::cout << "\n";
}

// ---------------------------------------------------------------
// Test 5: Prueba de Estabilidad de Integradores (M5)
// ---------------------------------------------------------------
void test_integrator_stability(IntegratorType type, const char *name)
{
    std::cout << "\n========================================================\n";
    std::cout << "PRUEBA 5: ESTABILIDAD DE INTEGRADORES (M5)\n";
    std::cout << "INTEGRADOR: " << name << "\n";
    std::cout << "OBJETIVO: Demostrar inestabilidad del Euler Explícito con dt alto.\n";
    std::cout << "========================================================\n";

    const float dt = 0.1f;                  // Paso de tiempo ALTO (Causa inestabilidad en Euler Explícito)
    const float INITIAL_FALL_SPEED = 15.0f; // Un valor grande para forzar el fallo

    body A(vec2(0.0f, 5.0f), vec2(0.0f, -INITIAL_FALL_SPEED), vec2(0.0f, 0.0f), 1.0f, 1.0f, 1.0f);
    // Cuerpo B (Estático): Suelo para colisionar. Posición Y=0.0
    body B(vec2(0.0f, 0.0f), vec2(0.0f, 0.0f), vec2(0.0f, 0.0f), 0.0f, 0.0f, 1.0f);

    std::vector<body> bodies = {A, B};
    vec2 gravedad_sim(0.0f, -9.81f);

    world stability_world(bodies, gravedad_sim, dt);

    // 1. Establecer el integrador para la prueba
    stability_world.integrador_actual = type; // Usar el nombre de la variable de world::hpp

    // 2. Asegurarse de que posicion_previa esté inicializada para Verlet
    stability_world.bodies[0].posicion_previa = stability_world.bodies[0].posicion;

    float max_height = 5.0f;
    bool exploded = false;

    // Ejecutar 50 ticks para observar el comportamiento
    for (int t = 0; t < 50; ++t)
    {
        stability_world.update();
        float current_y = stability_world.bodies[0].posicion.y;

        // Chequeo de explosión (valores no físicos)
        if (std::isnan(current_y) || std::isinf(current_y) || current_y > 20.0f)
        {
            exploded = true;
            break;
        }

        // Rastreamos la altura máxima de rebote
        if (current_y > max_height)
        {
            max_height = current_y;
        }
    }

    std::cout << "ESTADO FINAL:\n";

    if (exploded)
    {
        std::cout << "RESULTADO: FALLO - EXPLOSIÓN / INESTABILIDAD\n";
    }
    else
    {
        std::cout << "RESULTADO: ÉXITO - ESTABLE. Altura máxima de rebote: " << std::fixed << std::setprecision(4) << max_height << "m\n";
    }
    std::cout << "\n";
}

// ================================================================
// FUNCIÓN PRINCIPAL MAIN()
// ================================================================
int main()
{
    // --- M1: Pruebas de Constructores ---
    test_constructors();

    // --- M2/M3: Pruebas de Inicialización y Simulación Simple ---
    test_random_initialization();

    // --- M4: Pruebas de Colisiones ---
    test_elastic_collision();
    test_static_collision();

    // --- M5: Pruebas de Integradores ---
    test_integrator_stability(EULER_EXPLICIT, "EULER EXPLICITO (DEBE EXPLOTAR)");
    test_integrator_stability(EULER_SEMI_IMPLICIT, "EULER SEMI-IMPLICITO (ESTABLE)");
    test_integrator_stability(VERLET_POSITION, "VERLET POSITION (CONSERVADOR)");

    return 0;
}