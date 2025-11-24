#include "body.hpp"
#include "vec2.hpp"
#include "world.hpp"
// world(std::vector<body> &b, const vec2 &gravedad, float delta_time);
world::world() : bodies(), gravedad(), delta_time(0.0f) {}

world::world(const std::vector<body> &b_param, const vec2 &gravedad_param, float delta_time_param) : bodies(b_param), gravedad(gravedad_param), delta_time(delta_time_param) {}

void world::update()
{
    for (body &b : bodies)
    {
        // 1. Cálculo de Aceleración Total
        vec2 aceleracion_total = b.aceleracion + gravedad;

        // 2. Actualización de Velocidad
        b.velocidad = b.velocidad + (aceleracion_total * delta_time);

        // 3. Actualización de Posición
        b.posicion = b.posicion + (b.velocidad * delta_time);

        // 4. Limpieza
        b.aceleracion = vec2(0.0f, 0.0f);
    }
}