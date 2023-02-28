#include <libv/core/miscmath.hpp>
#include <libv/lma/lma.hpp>
#include <libv/graphic/viewer_context.hpp>
#include <libv/geometry/rotation.hpp>

#include <GL/gl.h>
#include <functional>
#include <thread>
#include <atomic>

  template<class T> using vector = std::vector<T,Eigen::aligned_allocator<T>>;
  using namespace Eigen;
  using namespace lma;

  Vector3d on_sphere(double theta, double phi, double r)
  {
    return {r*std::cos(theta)*std::cos(phi),
           r*std::cos(theta)*std::sin(phi),
           r*std::sin(theta)};
  }

  auto draw_points = [](auto& points, auto& rotate)
  {
    v::viewer(0).layer(2).clear().pen_color(v::red).pen_width(3).point_style(v::Round).add_opengl([&]()
    {
      glBegin(GL_POINTS);
      for(auto& p : points)
        glVertex3dv((rotate*p).eval().data());
      glEnd();
    }).update();
  };

  auto draw_sphere = [](auto& sphere, auto& rotate)
  {
    v::viewer(0).layer(1).clear().pen_color(v::blue).pen_width(1).point_style(v::Round).add_opengl([&]()
    {
      for(double theta = -M_PI/2.0 ; theta < M_PI/2.0 ; theta+=0.1)
      {
        glBegin(GL_LINE_LOOP);
        for(double phi = -M_PI ; phi < M_PI ; phi+=0.1)
          glVertex3dv((rotate*on_sphere(theta,phi,sphere[3])).eval().data());
        glEnd();
      }

      for(double phi = -M_PI ; phi < M_PI ; phi+=0.1)
      {
        glBegin(GL_LINE_STRIP);
          for(double theta = -M_PI/2.0 ; theta < M_PI/2.0 ; theta+=0.1)
         glVertex3dv((rotate*on_sphere(theta,phi,sphere[3])).eval().data());
        glEnd();
      }
    }).update();
  };

  enum State { before, after0, after1, after2 };

  auto draw_text = [](State state)
  {
    std::string str = "";
    if (state == after0)
      str = "  Optimise Sphere";
    if (state == after1)
      str = "  Optimise Sphere + Points 3D";
    else if(state == after2)
     str ="  Optimise Sphere + Points 3D + Contrainte";
    v::viewer(0).layer(0).clear().pen_color(v::red).font_size(30).font_style(v::Bold).add_text(40,50,str).update();
  };

  std::atomic<bool> stop_thread(false);
  void draw(Vector4d& sphere, vector<Vector3d>& points, State& state)
  {
    v::start_viewers();
    v::set_interval(5);
    Matrix3d rotate = Matrix3d::Identity();

    while(!stop_thread.load(std::memory_order_relaxed))
    {
      v::apply_rotation(rotate,{0.0,0.0,0.001});
      draw_text(state);
      draw_sphere(sphere,rotate);
      draw_points(points,rotate);
      
      v::tempo(5);
    }
  };


//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------

  auto optimise = [](Vector4d& sphere, vector<Vector3d>& points, const vector<Vector3d>& refs, int mode)
  {
    
    struct DistanceToSphere1
    {
      const Vector3d& point;
      DistanceToSphere1(const Vector3d& point_):point(point_){}

      bool operator()(const Vector4d& sphere, double& error) const
      {
        error = (sphere.head<3>()-point).norm() - sphere[3];
        return true;
      }
    };

    struct DistanceToSphere
    {
      bool operator()(const Vector4d& sphere, const Vector3d& point, double& error) const
      {
        error = (sphere.head<3>()-point).norm() - sphere[3];
        return true;
      }
    };

    struct DistanceTo3DPoint
    {
      const Vector3d& ref;
      DistanceTo3DPoint(const Vector3d& ref_):ref(ref_){}

      bool operator()(const Vector3d& point, double& error) const
      {
        error = (ref - point).norm();
        return true;
      }
    };

    Solver<DistanceToSphere1,DistanceToSphere,DistanceTo3DPoint> solver(1,50);

    if (mode==0)
    for(Vector3d& p : points)
      solver.add(DistanceToSphere1(p),&sphere);

    if (mode>0)
    for(Vector3d& p : points)
      solver.add(DistanceToSphere(),&sphere,&p);

    if (mode==2)
    for(size_t i = 0 ; i < refs.size() ; ++i)
      solver.add(DistanceTo3DPoint(refs[i]),&points[i]);

    solver.solve(DENSE_SCHUR,enable_verbose_output());
  };

//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------


int main()
{
  v::viewer(0).size(800,660);
  Vector4d sphere;
  sphere << 0,0,0,100;

  vector<Vector3d> points,refs;

  for(double theta = -M_PI/2.0 ; theta < M_PI/2.0 ; theta+=0.1)
    for(double phi = -M_PI ; phi < M_PI ; phi+=0.1)
      points.emplace_back(
        on_sphere(theta,phi,sphere[3])
        );

  refs = points;

  for(auto& p : points) p += Vector3d::Random()*50.0;

  sphere << 0,0,0,50;

  State state = before;
  std::thread th(std::bind(draw,boost::ref(sphere),boost::ref(points),boost::ref(state)));



  getchar();
  optimise(sphere,points,refs,0);

  state = after0;
  getchar();
  optimise(sphere,points,refs,1);
  state = after1;
  getchar();
  optimise(sphere,points,refs,2);
  state = after2;

  v::wait_viewers();
  stop_thread.store(true, std::memory_order_relaxed);
  th.join();

  return 0;
}
