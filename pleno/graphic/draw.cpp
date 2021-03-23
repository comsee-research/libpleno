#include "draw.h"

#include <GL/gl.h>

/*
 * @Brief glAddPoint Draw 3D point using OpenGL
 */
void glAddPoint(const P3D& p)
{
    glVertex3dv(p.data());
}

/////////////////////////////////glAddLine//////////////////////////////////////////////////////////
/*
 * @Brief glAddLine Draw a line using OpenGL
 */
void glAddLine(const P3D& p1, const P3D& p2)
{
    glVertex3dv(p1.data());
    glVertex3dv(p2.data());
}

/*
    @Brief glAddLine Draw a line using OpenGL
*/
void glAddLine(const Ray3D& r, double c)
{
    glAddLine(r.origin(), r(c));
}

/////////////////////////////////draw_axis//////////////////////////////////////////////////////////
/*
 * @Brief draw_axis
    tres mal pensee (j etais jeune)
 */
void draw_axis(const Pose& pose, double s = 1.0)
{
    const P3DS base {P3D{0.0, 0.0, 0.0},
               P3D{  s, 0.0, 0.0},
               P3D{0.0,   s, 0.0},
               P3D{0.0, 0.0,   s}};

    //Expressing base in WORLD
    int i = 0;
    for (const P3D& r : base)
    {
        colorize(i++, 1.0);
        glAddLine(pose.translation(), from_coordinate_system_of(pose, r));
    }
}

////////////////////////////////colorize////////////////////////////////////////////////////////////
/*
    @Brief colorize
    tres mal pensee (j etais jeune)
*/
void colorize(int d, double f)
{
    switch (d)
    {
        case 1:
            glColor3f(f, 0.0, 0.0); //x axis: red
            break;
        case 2:
            glColor3f(0.0, f, 0.0); //y axis: green
        break;
        case 3:
            glColor3f(0.0, 0.0, f); //z axis: blue
        break;
    }
}

/////////////////////////////////fill_form//////////////////////////////////////////////////////////
/*
    @Brief fill_form Fill a form in OpenGL
*/
void fill_form(const Quad3D& q)
{
    for (auto& p : q)
        glVertex3dv(p.data());
}
