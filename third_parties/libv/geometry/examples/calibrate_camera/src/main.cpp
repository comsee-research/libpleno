/**

\example

Étalonne les paramètres intrinsèques d'une caméra avec différents modèles.

Ce programme s'utilise en ligne de commande avec plein d'arguments :
- la largeur de la grille ;
- la hauteur de la grille ;
- des photos de mires dans des positions diverses.

Par exemple : `calibrate_camera 9 6 camera_1/mire_*.jpg`

La mire doit être compatible avec la fonction findChessboardCorners() d'OpenCV.

Sont déjà implémentés :
- le modèle sténopé OpenCV ;
- le modèle unifié Libv.

\author Alexis Wilhelm (2018)

\cond

*/

#include <libv/geometry/camera_model.hpp>
#include <libv/geometry/pose.hpp>
#include <libv/geometry/rotation.hpp>
#include <libv/lma/lma.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

// Pour optimiser nos objets, il faut définir les fonctions Size, apply_increment() et apply_small_increment() pour chaque type.
// - Size donne la dimension d'un objet.
// - apply_increment() modifie un objet à chaque itération de l'optimisation.
// - apply_small_increment() permet de calculer rapidement la jacobienne de la fonction d'erreur (si on utilise la dérivation numérique).
// Ici, on les définit pour UnifiedCameraModel et Pose, les deux types qu'on veut optimiser.
namespace lma {

#define _(_) void apply_small_increment(X &o, double d, const v::numeric_tag<_> &, const Adl &)

#define X v::UnifiedCameraModel
_(0){ o.focal[0] += d; }
_(1){ o.focal[1] += d; }
_(2){ o.center[0] += d; }
_(3){ o.center[1] += d; }
_(4){ o.xi += d; }
template<> struct Size<X> { enum {value = 5}; };
void apply_increment(X &o, const double *d, const Adl &)
#undef X
{
  o.focal[0] += d[0];
  o.focal[1] += d[1];
  o.center[0] += d[2];
  o.center[1] += d[3];
  o.xi += d[4];
}

#define X v::Pose
_(0){ o.translation()[0] += d; }
_(1){ o.translation()[1] += d; }
_(2){ o.translation()[2] += d; }
_(3){ v::apply_small_rotation(o.rotation(), {d, 0, 0}); }
_(4){ v::apply_small_rotation(o.rotation(), {0, d, 0}); }
_(5){ v::apply_small_rotation(o.rotation(), {0, 0, d}); }
template<> struct Size<X> { enum {value = 6}; };
void apply_increment(X &o, const double *d, const Adl &)
#undef X
{
  o.translation()[0] += d[0];
  o.translation()[1] += d[1];
  o.translation()[2] += d[2];
  v::apply_rotation(o.rotation(), {d[3], 0, 0});
  v::apply_rotation(o.rotation(), {0, d[4], 0});
  v::apply_rotation(o.rotation(), {0, 0, d[5]});
}

#undef _

}

namespace error {

// Une contrainte utilisée dans l'optimisation.
// Évalue l'erreur de reprojection d'un point de la grille.
struct Unified
{
  const Eigen::Vector2d p2; // Un point de la grille détecté dans une image.
  const Eigen::Vector3d p3; // Le point correspondant dans le repère de la mire.

  bool operator()
  ( const v::UnifiedCameraModel &model // Le modèle de caméra qu'on est en train d'étalonner.
  , const v::Pose &pose // La pose de la mire dans cette vue.
  , Eigen::Vector2d &error // [out] L'erreur de reprojection de ce point.
  ) const
  {
    Eigen::Vector2d p;
    if(!model.project(from_coordinate_system_of(pose, p3), p))
    {
      return false;
    }
    error = p - p2;
    return true;
  }
};

}

namespace {

static void error_opencv(
  const std::vector<std::vector<cv::Vec2f>> &p2,
  const std::vector<std::vector<cv::Vec3f>> &p3,
  const cv::Mat3d &rotations,
  const cv::Mat3d &translations,
  const cv::Mat1d &model,
  const cv::Mat1d &distortion,
  double &mean,
  double &stddev)
{
  mean = 0;
  stddev = 0;
  size_t successes = p2.size() * p2.front().size();
  std::vector<cv::Vec2f> projected;
  for(size_t i = 0; i < p2.size(); ++i)
  {
    projected.clear();
    projectPoints(p3[i], rotations(i), translations(i), model, distortion, projected);
    for(size_t k = 0; k < p2[i].size(); ++k)
    {
      const double sq_error = Eigen::Vector2d(projected[k][0] - p2[i][k][0], projected[k][1] - p2[i][k][1]).squaredNorm();
      mean += std::sqrt(sq_error);
      stddev += sq_error;
    }
  }
  mean /= successes;
  stddev = std::sqrt(stddev / successes - mean * mean);
}

static void error_unified(
  const std::vector<std::vector<cv::Vec2f>> &p2,
  const std::vector<std::vector<cv::Vec3f>> &p3,
  const v::Pose *poses,
  const v::UnifiedCameraModel &model,
  double &mean,
  double &stddev,
  size_t &failures)
{
  mean = 0;
  stddev = 0;
  failures = 0;
  size_t successes = 0;
  for(size_t i = 0; i < p2.size(); ++i)
  {
    for(size_t k = 0; k < p2[i].size(); ++k)
    {
      Eigen::Vector2d projected;
      const bool good = model.project(from_coordinate_system_of(poses[i], {p3[i][k][0], p3[i][k][1], p3[i][k][2]}), projected);
      if(good)
      {
        const double sq_error = (projected - Eigen::Vector2d(p2[i][k][0], p2[i][k][1])).squaredNorm();
        mean += std::sqrt(sq_error);
        stddev += sq_error;
        ++successes;
      }
      else
      {
        ++failures;
      }
    }
  }
  mean /= successes;
  stddev = std::sqrt(stddev / successes - mean * mean);
}

}

int main(int argc, char **argv)
{
  if(argc < 5)
  {
    std::cerr << "Usage: calibrate_camera GRID_WIDTH GRID_HEIGHT IMAGE..." << std::endl;
    return EXIT_FAILURE;
  }

  // La taille de la grille.
  // Attention : on compte les coins entre quatre carrés du damier, pas les carrés eux-même.
  const cv::Size size(atoi(argv[1]), atoi(argv[2]));

  std::cerr << "-- Calibrating using " << (argc - 3) << " views of a " << size << " chessboard..." << std::endl;

  // Trouve la mire dans les images.
  // On doit bien trouver tous les points de la grille.
  // Si on ne les trouve pas dans une image, on ignore cette image.
  cv::Mat image;
  std::vector<cv::Vec2f> p2_;
  std::vector<std::vector<cv::Vec2f>> p2;
  for(int i = 3; i < argc; ++i)
  {
    image = cv::imread(argv[i]);
    if(image.empty())
    {
      std::cerr << argv[i] << ": error in cv::imread." << std::endl;
      continue;
    }
    if(!findChessboardCorners(image, size, p2_))
    {
      std::cerr << argv[i] << ": error in cv::findChessboardCorners." << std::endl;
      continue;
    }
    p2.emplace_back(move(p2_));
  }
  // Les mires sont détectées.

  // Initialise une grille arbitraire.
  // On place un point tous les mètres, sans se soucier des dimensions réelles de la mire.
  // On doit la dupliquer pour chaque image pour pouvoir la donner à calibrateCamera().
  std::vector<cv::Vec3f> p3_;
  for(int y = 0; y < size.height; ++y)
  {
    for(int x = 0; x < size.width; ++x)
    {
      p3_.emplace_back(x, y, 0);
    }
  }
  const std::vector<std::vector<cv::Vec3f>> p3(p2.size(), p3_);
  // La grille est générée.

  // Étalonne la caméra avec OpenCV.
  // Les paramètres et les poses trouvés nous serviront à initialiser nos modèles pour que l'optimisation se passe bien.
  cv::Mat1d matrix, distortion;
  cv::Mat3d rotation, translation;
  calibrateCamera(p3, p2, image.size(), matrix, distortion, rotation, translation);

  double mean = -1, stddev = -1;
  error_opencv(p2, p3, rotation, translation, matrix, distortion, mean, stddev);
  std::cout << "cv::calibrateCamera:"
    << "\n\tfocal:\t" << matrix.diag()(cv::Range(0, 2), cv::Range::all()).t()
    << "\n\tcenter:\t" << matrix.col(2)(cv::Range(0, 2), cv::Range::all()).t()
    << "\n\tdist:\t" << distortion
    << "\n\terror:\t" << mean << " ±" << stddev
    << std::endl;

  // Le modèle de caméra qu'on veut étalonner.
  // On l'initialise avec l'étalonnage OpenCV, et ensuite il sera optimisé.
  v::UnifiedCameraModel model(matrix(0,0), matrix(1,1), matrix(0,2), matrix(1,2), 0);

  // La pose de la mire dans chaque vue.
  // Elles ne nous intéressent pas, mais on en a besoin pendant l'optimisation.
  // Il faut les initialiser, sinon l'optimisation converge vers une configuration dégénérée avec la mire du mauvais côté de la caméra.
  v::Pose poses[p2.size()];

  lma::Solver<error::Unified> solver(1e-3, 1e3, 1);
  for(size_t i = 0; i < p2.size(); ++i)
  {
    // Initialise la pose de la mire dans cette vue.
    poses[i].translation({translation(i)[0], translation(i)[1], translation(i)[2]});
    v::apply_rotation(poses[i].rotation(), {rotation(i)[0], rotation(i)[1], rotation(i)[2]});

    // Utilise les correspondances entre p2 et p3 dans l'optimisation.
    for(size_t k = 0; k < p2[i].size(); ++k)
    {
      solver.add(error::Unified{{p2[i][k][0], p2[i][k][1]}, {p3[i][k][0], p3[i][k][1], p3[i][k][2]}}, &model, &poses[i]);
    }
  }

  // Optimise le modèle de caméra.
  // On optimise aussi les poses de la mire, mais elles ne nous intéressent pas.
  // On peut afficher le détail de l'optimisation en ajoutant lma::enable_verbose_output() comme deuxième argument.
  solver.solve(lma::DENSE);

  size_t failures = -1;
  error_unified(p2, p3, poses, model, mean, stddev, failures);
  std::cout << "v::UnifiedCameraModel:" << std::setprecision(20)
    << "\n\tfocal:\t" << model.focal.transpose()
    << "\n\tcenter:\t" << model.center.transpose()
    << "\n\tdist:\t" << model.xi
    << std::setprecision(5)
    << "\n\terror:\t" << mean << " ±" << stddev << " (" << failures << " failures)"
    << std::endl;
}

/// \endcond
