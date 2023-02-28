#include <libv/geometry/pose.hpp>
#include <libv/geometry/rotation.hpp>
#include <libv/geometry/camera_model.hpp>

#include <libv/graphic/viewer_context.hpp>
#include <GL/gl.h>
#include <functional>

#include <iostream>

#include <libv/lma/lma.hpp>

namespace lma
{
  template<> struct Size<v::Pose> { enum {value = 6}; };
  template<> struct Size<v::UnifiedCameraModel> { enum {value = 5}; };

  template<int I> void apply_small_increment(Eigen::Matrix3d& rotation, double d, v::numeric_tag<I>, const Adl&)
  {
    v::apply_small_rotation(rotation,Eigen::Vector3d(I==0?d:0.0,I==1?d:0.0,I==2?d:0.0));
  }

  template<int I> void apply_small_increment(v::Pose& o, double d, v::numeric_tag<I>, const Adl&)
  {
    if (I<3)
      o.translation()[I] += d;
    else
      v::apply_small_rotation(o.rotation(),Eigen::Vector3d(I==3?d:0.0,I==4?d:0.0,I==5?d:0.0));
  }

  inline void apply_increment(v::Pose& vue, const double delta[6], const Adl&)
  {
    for(size_t i = 0 ; i < 3 ; ++i)
      vue.translation()[i] += delta[i];
    v::apply_rotation(vue.rotation(), Eigen::Vector3d(delta[3],delta[4],delta[5]));
  }
  
  inline void apply_increment(v::UnifiedCameraModel& model, const double delta[5], const Adl&)
  {
    model.focal.x() += delta[0];
    model.focal.y() += delta[1];
    model.center.x() += delta[2];
    model.center.y() += delta[3];
    model.xi += delta[4];
  }

  template<int I> void apply_small_increment(v::UnifiedCameraModel& model, double d, v::numeric_tag<I>, const Adl&)
  {
    if (I==0) 
      model.focal.x() += d;
    else if(I==1)
      model.focal.y() += d;
    else if(I==2)
      model.center.x() += d;
    else if(I==3)
      model.center.y() += d;
    else if(I==4)
      model.xi += d;
  }
}


struct Obs
{
  Eigen::Vector2d coord;
  int ipose;
  int i3d;
  Obs(){}
  Obs(const Eigen::Vector2d& v2d, int ipose_, int i3d_):coord(v2d),ipose(ipose_),i3d(i3d_){}
};

template<typename T> using AlignedVector = std::vector<T,Eigen::aligned_allocator<T>>;
typedef AlignedVector<Eigen::Vector3d> P3DS;
typedef AlignedVector<Obs> P2DS;
typedef AlignedVector<v::Pose> Poses;

void draw_pose(const v::Pose& pose, double d)
{

  glLineWidth(2);
  Eigen::Vector3d o1(0,0,0);
  Eigen::Vector3d v1(d,0,0);
  Eigen::Vector3d v2(0,d,0);
  Eigen::Vector3d v3(0,0,d);

  o1 = pose.rotation() * o1 + pose.translation();
  v1 = pose.rotation() * v1 + pose.translation();
  v2 = pose.rotation() * v2 + pose.translation();
  v3 = pose.rotation() * v3 + pose.translation();

  glBegin(GL_LINES);
  glColor3f(1.0,0.0,0.0);
  glVertex3f(o1.x(),o1.y(),o1.z());
  glVertex3f(v1.x(),v1.y(),v1.z());

  glColor3f(0.0,1.0,0.0);
  glVertex3f(o1.x(),o1.y(),o1.z());
  glVertex3f(v2.x(),v2.y(),v2.z());

  glColor3f(0.0,0.0,1.0);
  glVertex3f(o1.x(),o1.y(),o1.z());
  glVertex3f(v3.x(),v3.y(),v3.z());
  glEnd();
}

void draw(const P3DS& p3ds, const Poses& poses, const float d)
{
  glColor3f(1,0,0);
  glPointSize(2);
  glBegin(GL_POINTS);
  for(auto& p : p3ds)
    glVertex3dv(p.data());
  glEnd();

  for(auto& pose : poses)
    draw_pose(pose,d);
}

bool projection(const v::UnifiedCameraModel& model, const v::Pose& pose, const Eigen::Vector3d& p3d, Eigen::Vector2d& p2d)
{
  Eigen::Vector3d p3dc = v::to_coordinate_system_of(pose,p3d);
  if (p3dc.z()<0.1) return false;
  return model.project(p3dc, p2d);
}

bool error(const v::UnifiedCameraModel& model, const v::Pose& pose, const Eigen::Vector3d& p3d, const Eigen::Vector2d& obs, Eigen::Vector2d& residual)
{
  Eigen::Vector2d p2d;
  bool success = projection(model,pose,p3d,p2d);
  residual = p2d - obs;
  return success;
}

struct Error
{
  const Eigen::Vector2d& obs;
  Error(const Eigen::Vector2d& obs_):obs(obs_){}

  bool operator()(const v::UnifiedCameraModel& model, const v::Pose& pose, const Eigen::Vector3d& p3d, Eigen::Vector2d& residual) const
  {
    return error(model,pose,p3d,obs,residual);
  }
};

int main()
{
  v::UnifiedCameraModel model(800,800,320,240,2.0);

  P3DS p3ds(1000);
  for(auto& p : p3ds)
    p = Eigen::Vector3d().Random()*50.0 + Eigen::Vector3d(0,0,400);

  Poses poses(10);
  for(auto& pose : poses)
    pose.translation(Eigen::Vector3d().Random()*100.0);

  P2DS p2ds;

  for(size_t i = 0 ; i < p3ds.size() ; ++i)
    for(size_t j = 0 ; j < poses.size() ; ++j)
    {
      Eigen::Vector2d p2d;
      if (projection(model,poses.at(j),p3ds.at(i),p2d))
      {
        p2ds.emplace_back(p2d,j,i);
      }
      else
      {
        std::cout << " p3d " << i << " avec la pose " << j << " ne se projette pas " << std::endl;
      }
    }

  for(auto& p : p3ds)
    p += Eigen::Vector3d().Random()*50.0;

  v::viewer(0).size(800,600).title("3D").interaction_mode(v::INTERACTION_CAD).add_opengl(std::bind(draw,p3ds,poses,30)).update();

  std::cout << " MODEL ORIGINAL : " << model.focal.transpose() << " ; " << model.center.transpose() << " ; " << model.xi << std::endl;

  model = v::UnifiedCameraModel(750,850,300,200,1.5);

  std::cout << " MODEL BRUITER : " << model.focal.transpose() << " ; " << model.center.transpose() << " ; " << model.xi << std::endl;
  
  lma::Solver<Error> solver(-1,500,0.999999999);

  for(auto& obs : p2ds)
    solver.add(Error(obs.coord),&model,&poses.at(obs.ipose),&p3ds.at(obs.i3d));

  solver.solve(lma::DENSE_SCHUR,lma::enable_verbose_output());
  // changement de repère monde -> caméra

  // std::cout << " Coordonnées du point dans l'image : " << p2d.transpose() << std::endl;
  std::cout << " MODEL APRES OPTIMISATION : " << model.focal.transpose() << " ; " << model.center.transpose() << " ; " << model.xi << std::endl;

  v::viewer(0).clear().size(800,600).title("3D").interaction_mode(v::INTERACTION_CAD).add_opengl(std::bind(draw,p3ds,poses,30)).update();

  
  v::ViewerContext view = v::viewer(1).title("2D").size(640,480).pen_width(5).pen_color(v::red);
  for(auto& p : p2ds)
  {
    if (p.ipose==0)
      view.add_point(p.coord.x(),p.coord.y());
  }
  view.update();

   v::wait_viewers();

  return 0;
}
