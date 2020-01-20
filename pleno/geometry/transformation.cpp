#include "transformation.h"

#include "processing/tools/range.h"

Eigen::Matrix<double, 3, 3> Transformation::operator()() const
{
    return total_matrix;
}

// TODO: dégager les calculs par zero
void Transformation::reset()
{
    total_matrix = k() * Eigen::Matrix<double, 3, 3>::Identity() * k().inverse();
}

Eigen::Matrix<double, 3, 3> Transformation::k() const // pinhole mode
{
    // TODO: et si on modifie les paramètres de la caméra ? on est niqué car k change pas !

    static Eigen::Matrix<double, 3, 3> m;

    static bool init = false;
    if (!init)
    {
        m << ucm.focal.x(),           0.0, ucm.center.x(),
                       0.0, ucm.focal.y(), ucm.center.y(),
                       0.0,           0.0,            1.0;
    }
    init = true;
    return m;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * @Brief exponential_map
 * Compute the exponential map according to lie algebra rules
 * @range the lie generators
 * @x the state vector
 */
Eigen::Matrix<double, 3, 3> exponential_map(auto range, const Eigen::VectorXd& x)
{
    Eigen::Matrix<double, 3, 3> exp_map = Eigen::Matrix<double, 3, 3>::Zero();

    int i = 0;
    for (auto& r : range)
    {
        for (std::size_t s = 0; s < r.size(); ++s)
        {
            if ( r(s) != 0 or x(i) != 0 )
                exp_map(s) += x(i) * r(s);
        }
        ++i;
    }

    Eigen::Matrix<double, 3, 3> TMP = exp_map;

    // comes from eigen unsupported so it is not stable
    // exp_map = exp_map.exp();

    // compute the exponential map
    const std::array<double, 5> factorials {{1.0, 1.0, 2.0, 6.0, 24.0}};
    const std::array<Eigen::Matrix<double, 3, 3>, 5> trucs {{Eigen::Matrix<double, 3, 3>::Identity(),
                                                                                                 TMP,
                                                                                           TMP * TMP,
                                                                                     TMP * TMP * TMP,
                                                                             TMP * TMP * TMP * TMP}};

    Eigen::Matrix<double, 3, 3> results = Eigen::Matrix<double, 3, 3>::Zero();
    for (std::size_t k = 0; k < factorials.size(); ++k)
    {
        results += (1.0 / factorials[k]) * trucs[k];
    }

    return results;
}

// Using Eade lie algebra bases
const AlignedVector<Eigen::Matrix<double, 3, 3>>& lie_algebra_generators()
{
    static AlignedVector<Eigen::Matrix<double, 3, 3>> generators(6);

    static bool init = false;
    if (!init)
    {
        generators[0] << 0,-1, 0,   1, 0, 0,   0, 0, 0; // rotation
        generators[1] << 0, 0, 1,   0, 0, 0,   0, 0, 0; // translation u
        generators[2] << 0, 0, 0,   0, 0, 1,   0, 0, 0; // translation v
        generators[3] << 1, 0, 0,   0, 1, 0,   0, 0, 0; // scale
        generators[4] << 1, 0, 0,   0,-1, 0,   0, 0, 0; // stretch
        generators[5] << 0, 1, 0,   1, 0, 0,   0, 0, 0; // shear

        init = true;
    }

    return generators;
}

// Compute the exponential map of a matrix (according to the Lie algebra)
Eigen::Matrix<double, 3, 3> compute_exponential_map(const Eigen::VectorXd& x, int k)
{
    return exponential_map( range(lie_algebra_generators(), 0, k), x);
}

Eigen::Matrix<double, 3, 3> get_exponential_map(const Transformation& t, const Eigen::VectorXd& a)
{
    Mode mode = t.mode;

    if (mode == SO)
        return compute_exponential_map(a, 1);
    if (mode == SE)
        return compute_exponential_map(a, 3);
    if (mode == SIM)
        return compute_exponential_map(a, 4);
    if (mode == AFF)
        return compute_exponential_map(a, 6);

    assert("Error: get_exponential_map: ");
    std::cout << "arrête de désactiver les assertions" << std::endl;
    abort();
}

// the jacobian is a  9 * mode matrix
// where mode is the number a degree of liberty
const Eigen::MatrixXd get_transformation_jacobian(const Transformation& t)
{
    const size_t mode = t.mode;
    Eigen::MatrixXd jacobian(9, mode);

    // the tmp matrix contains the transpose of the genreators
    Eigen::MatrixXd tmp;
    tmp.setZero();

    std::size_t i = 0;
    for (const auto& generators : range(lie_algebra_generators(), 0, mode))
    {
        tmp = generators.transpose();

        for (std::size_t s = 0; s < tmp.size(); ++s)
        {
            jacobian(i) = tmp(s);
            ++i;
        }
    }
        
    return jacobian;
}
