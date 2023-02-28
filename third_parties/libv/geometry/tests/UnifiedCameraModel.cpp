/**

\file

Vérifie que les fonctions UnifiedCameraModel::project() et UnifiedCameraModel::raytrace() sont bien l'inverse l'une de l'autre.

\author Alexis Wilhelm
\copyright 2017 Institut Pascal
\privatesection

*/

#include <libv/core/logger.hpp>
#include <libv/core/test.hpp>
#include <libv/geometry/camera_model.hpp>
#include <libv/geometry/io.hpp>

using namespace v;

int main(int, char **)
{
  double max_error = 0;
  const UnifiedCameraModel cameras[] {
    {100, 100, 100, 100, .5},
    {100, 100, 100, 100, 1},
    {100, 100, 100, 100, 1.5},
    };
  for(const UnifiedCameraModel &camera: cameras)
  {
    size_t raytrace_failure_count = 0;
    size_t project_failure_count = 0;
    for(int i = 0; i < 200; ++i)
    {
      for(int j = 0; j < 200; ++j)
      {
        try
        {
          const Eigen::Vector2d p1 {j, i};
          Eigen::Vector3d p2;
          Eigen::Vector2d p3;
          camera.raytrace(p1, p2);
          const bool ok = camera.project(p2, p3);
          if(ok)
          {
            const double error = (p1 - p3).squaredNorm();
            if(error > max_error)
            {
              max_error = error;
            }
          }
          else
          {
            ++project_failure_count;
          }
        }
        catch(const std::domain_error &)
        {
          ++raytrace_failure_count;
        }
      }
    }
    V_DEBUG
      << " camera=" << camera
      << " raytrace=" << raytrace_failure_count
      << " project=" << project_failure_count
      ;
  }
  V_DEBUG << " max_error=" << max_error;
  V_TEST_LT(max_error, 1e-25);
}
