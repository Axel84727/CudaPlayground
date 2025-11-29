#pragma once
struct vec2
{
    float x;
    float y;

    vec2();

    vec2(float x_val, float y_val);
};

vec2 operator+(const vec2 &left, const vec2 &right);
vec2 operator*(float left, const vec2 &right);
vec2 operator*(const vec2 &left, float right);
vec2 operator-(const vec2 &left, const vec2 &right);
float dot(const vec2 &a, const vec2 &b);