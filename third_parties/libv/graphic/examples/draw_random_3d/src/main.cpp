/// \example
/// Draw random 3D shapes.
/// \author Alexis Wilhelm (2013)
/// \see viewers
/// \privatesection

#include <libv/graphic/viewer_context.hpp>
#include <GL/gl.h>

static const struct {
  float vertices[8][3];
  struct {
    int vertices[4];
    float normal[3];
  } faces[6];
} cube = {
  {
    {-1,-1,1},
    {-1,-1,-1},
    {-1,1,-1},
    {-1,1,1},
    {1,-1,1},
    {1,-1,-1},
    {1,1,-1},
    {1,1,1},
  },
  {
    {{0,1,2,3},{-1,0,0}},
    {{3,2,6,7},{0,1,0}},
    {{7,6,5,4},{1,0,0}},
    {{4,5,1,0},{0,-1,0}},
    {{5,6,2,1},{0,0,1}},
    {{7,4,0,3},{0,0,-1}},
  },
};

static void draw_scene()
{
  glEnable(GL_DEPTH_TEST);

  glPushMatrix();
  glLoadIdentity();
  glScalef(100, 100, 100);

  for(int i = 0; i < 6; ++i)
  {
    glBegin(GL_QUADS);
    glNormal3fv(cube.faces[i].normal);
    glColor3fv(cube.faces[i].normal);
    for(int j = 0; j < 4; ++j) glVertex3fv(cube.vertices[cube.faces[i].vertices[j]]);
    glEnd();
  }

  glPopMatrix();
}

int main()
{
  v::ViewerContext v = v::viewer()
    .title("Test")
    .background(v::dark_cyan)
    .pen_width(3)
    .interaction_mode(v::INTERACTION_CAD)
    ;
  v.layer(0).add_opengl(draw_scene).update();
  v.layer(1).add_rect(0, 0, 200, 200).update();
  v::wait_viewers();
}
