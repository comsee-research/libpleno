#pragma once

#include "types.h"

#include "geometry/ray.h"
#include "geometry/pose.h"


void glAddPoint(const P3D& p);

template<typename ... T>
void meta_glAddPoint(T&& ... a)
{
    std::initializer_list<int>{(static_cast<void>(glAddPoint(a)), 0)...};
}

void glAddLine(const P3D& p1,  const P3D& p2);
void glAddLine(const Ray3D& r, double = 1000.0);

// draw camera axis
void draw_axis(const Pose& pose, double d);

void colorize(int d, double f);
void fill_form(const Quad3D& q);

// Working only with rays
template<typename ... T>
void meta_glAddLine(T&& ... a)
{
    std::initializer_list<int>{(static_cast<void>(glAddLine(a)), 0)...};
}
