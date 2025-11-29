#include "vec2.hpp"

vec2::vec2() : x(0.0f), y(0.0f)
{
}

vec2::vec2(float val_x, float val_y) : x(val_x), y(val_y)
{
}

// src/vec2.cpp

// Suma de Vectores (vec2 + vec2)
vec2 operator+(const vec2 &left, const vec2 &right)
{
    return vec2(left.x + right.x, left.y + right.y);
}

// Multiplicación (Vector * Escalar)
vec2 operator*(const vec2 &left, float right)
{
    return vec2(left.x * right, left.y * right);
}

vec2 operator-(const vec2 &left, const vec2 &right)
{
    return vec2(left.x - right.x, left.y - right.y);
}

// Multiplicación (Escalar * Vector)
vec2 operator*(float left, const vec2 &right)
{
    return vec2(left * right.x, left * right.y);
}

float dot(const vec2 &a, const vec2 &b)
{
    return a.x * b.x + a.y * b.y;
}
