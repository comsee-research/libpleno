#include <libv/lma/lma.hpp>


#if 0
struct F { bool operator()(const Eigen::Vector2d&, const Eigen::Vector3d&, const Eigen::Vector2d&, double&) const { return true; }};
#endif
struct G { bool operator()(const Eigen::Vector3d&, const Eigen::Vector2d&, const Eigen::Vector2d&, double&) const { return true; }};


/*
 * (extrait d'une conversation par mail avec Datta)
 * L'utilité du test : s'assurer que LMA fonctionne lorsque l'opérateur () du foncteur contient 2 arguments d'un même type (ici vector2d).
 * Pour que ça fonctionne, il faut ordonner consécutivement les arguments du même type.
 * L'idéal serait évidemment que le test passe même quand les arguments identiques ne sont pas consécutifs...
 *
 * [SW] --> d'où les #if 0
 */

int main()
{
  Eigen::Vector2d a;
  Eigen::Vector3d b;
  Eigen::Vector2d c;

#if 0
  lma::Solver<lma::mpl::vector<Eigen::Vector2d,Eigen::Vector3d>,F>().add(F{},&a,&b,&c).solve(lma::DENSE);
#endif
  lma::Solver<lma::mpl::vector<Eigen::Vector2d,Eigen::Vector3d>,G>().add(G{},&b,&c,&c).solve(lma::DENSE);
  return 0;
}
