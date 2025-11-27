#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include "world.hpp"
// Función auxiliar para imprimir vec2
void print_vec2(const vec2 &v)
{
    std::cout << "(" << v.x << ", " << v.y << ")";
}

int main()
{
    std::cout << "--- PRUEBA DE CONSTRUCTORES ---" << std::endl;

    // 1. Pruebas de vec2

    // Constructor por Defecto: vec2() -> (0, 0)
    vec2 v_cero;
    std::cout << "1.1 Vec2 por Defecto: ";
    print_vec2(v_cero);
    std::cout << std::endl;

    // Constructor Parametrizado: vec2(float, float)
    vec2 v_pos_b1(10.0f, 5.0f);
    vec2 v_vel_b1(2.0f, 0.0f);
    vec2 v_accel_b1(0.0f, -9.8f);

    std::cout << "1.2 Vec2 Parametrizado: ";
    print_vec2(v_pos_b1);
    std::cout << std::endl;

    // 2. Pruebas de body

    // Masa e Inversa de Masa
    float masa_b1 = 50.0f;
    float inv_masa_b1 = 1.0f / masa_b1;

    // Constructor Parametrizado de body (usa const vec2& para las copias)
    body b1(v_pos_b1, v_vel_b1, v_accel_b1, masa_b1, inv_masa_b1, 1.0f);

    std::cout << "2. Body Inicializado:" << std::endl;
    std::cout << "   Posicion: ";
    print_vec2(b1.posicion);
    std::cout << ", Masa: " << b1.masa << ", Inv_M: " << b1.inv_mass << std::endl;

    // 3. Pruebas de world (Sobrecarga de Constructores)

    // a) Constructor por Defecto: world()
    world mundo_vacio;
    std::cout << "3.1 World por Defecto: Bodies = " << mundo_vacio.bodies.size();
    std::cout << ", Gravedad = ";
    print_vec2(mundo_vacio.gravedad);
    std::cout << std::endl;

    // b) Constructor Parametrizado: world(const std::vector<body>&, const vec2&, float)

    // Creamos el vector que pasaremos por referencia constante
    std::vector<body> cuerpos_iniciales;
    cuerpos_iniciales.push_back(b1);

    // Creamos un segundo cuerpo para hacer el vector más grande
    body b2(vec2(-10.0f, 0.0f), v_vel_b1, v_accel_b1, 5.0f, 1.0f / 5.0f, 0.3f);
    cuerpos_iniciales.push_back(b2);

    vec2 g_simulacion(0.0f, -10.0f);

    // Llama al constructor que copia el vector cuerpos_iniciales al miembro world::bodies
    world mi_mundo(cuerpos_iniciales, g_simulacion, 0.016f); // 0.016s es ~60 FPS

    std::cout << "3.2 World Parametrizado:" << std::endl;
    std::cout << "   Total Bodies: " << mi_mundo.bodies.size() << std::endl;
    std::cout << "   Gravedad: ";
    print_vec2(mi_mundo.gravedad);
    std::cout << ", Delta Time: " << mi_mundo.delta_time << std::endl;
    std::cout << "   Posicion del Body 1 copiado: ";
    print_vec2(mi_mundo.bodies[0].posicion);
    std::cout << std::endl;

    std::cout << "--- PRUEBA CON VALORES ALEATORIOS ---" << std::endl;

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

    std::cout << "----INICIALIZACION EXITOSA----" << std::endl;
    std::cout << "World inicializado con " << world1.bodies.size() << std::endl;
    std::cout << "El primer cuerpo esta en x:" << world1.bodies[0].posicion.x << " y: " << world1.bodies[0].posicion.y << std::endl;

    std::cout << "\n--- EJECUTANDO BUCLE DE SIMULACION ---\n";

    int max_ticks = 100;
    for (int t = 0; t < max_ticks; ++t)
    {
        world1.update();
        if (t % 10 == 0)
        {
            std::cout << "Tick " << t
                      << ": Pos Y -> " << world1.bodies[0].posicion.y
                      << " | Vel Y -> " << world1.bodies[0].velocidad.y << std::endl;
        }
    }
    std::cout << "\n--- RESULTADO FINAL ---\n";
    std::cout << "Cuerpo 0: Posicion Final Y: " << world1.bodies[0].posicion.y << std::endl;
    // Si la gravedad es -9.81, el valor de 'Pos Y' y 'Vel Y' deben ser menores que el inicial.
    return 0;
}