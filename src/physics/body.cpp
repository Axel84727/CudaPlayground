#include "physics/body.hpp"

body::body(const vec2 &position_param,
           const vec2 &velocity_param,
           const vec2 &acceleration_param,
           float mass_param,
           float inv_mass_param,
           float radius_param,
           float restitution_param,
           float damping_param,
           float friction_param)
    : position(position_param),
      previous_position(position_param),
      velocity(velocity_param),
      acceleration(acceleration_param),
      mass(mass_param),
      inv_mass(inv_mass_param),
      radius(radius_param),
      damping(damping_param),
      friction(friction_param),
      restitution(restitution_param)
{
    if (mass_param <= 0.0f)
    {
        inv_mass = 0.0f;
    }
}

body::body()
    : position(),
      previous_position(),
      velocity(),
      acceleration(),
      mass(1.60f),
      inv_mass(1.0f / 1.60f),
      radius(1.80f),
      damping(0.10f),
      friction(0.50f),
      restitution(0.85f)
{
}