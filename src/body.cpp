// body(const vec2 &posicion, const vec2 &velocidad, const vec2 &aceleracion, float masa, float inv_mass, float radio);
#include "body.hpp"
body::body(const vec2 &posicion,
           const vec2 &velocidad,
           const vec2 &aceleracion,
           float masa,
           float inv_mass,
           float radio) : posicion(posicion), velocidad(velocidad), aceleracion(aceleracion), masa(masa), inv_mass(inv_mass), radio() {}

body::body() : posicion(), velocidad(), aceleracion(), masa(0.0f), inv_mass(0.0f), radio(0.0f) {}