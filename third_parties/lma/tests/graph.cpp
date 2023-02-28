
#include <libv/lma/graph/graph.hpp>

using namespace lma;

using Pose = Eigen::Matrix<double,6,1>;
using Point3D = Eigen::Matrix<double,3,1>;

struct Poses : Parameter<Pose>
{
  AlignedVector<Pose> poses;

  void cover(auto&)
  {
  	poses = {Pose::Random(),Pose::Random()};
  }
};


struct Points3D : Parameter<Point3D>
{
  AlignedVector<Point3D> points;

  void cover(auto& graph)
  {
  	Poses& poses = graph.node();
  	graph.cover(poses);
  	points = {Point3D::Random(),Point3D::Random(),Point3D::Random()};
  }
};


struct Reprojection
{
  bool operator()(const Pose& pose, const Point3D& point3d, double (&res) [2]) const
  {
  	res[0] = pose.norm() + point3d.norm();
  	res[1] = pose.squaredNorm() + point3d.squaredNorm();
  	return true;
  }
};

struct Reprojections : Constraint<Reprojection>
{
  void cover(auto& graph)
  {
  	Poses& poses = graph.node();
  	Points3D& points3d = graph.node();
    graph.cover(points3d);

  	for(Pose& pose : poses.poses)
      for(auto& p3d : points3d.points)
        add(Reprojection(),&pose,&p3d);
  }
};


int main()
{
  Graph<Constraints(Reprojections),Parameters(Poses,Points3D)>().solve(DENSE,enable_verbose_output());
}