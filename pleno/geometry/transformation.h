#pragma once //from charlib

#include <Eigen/Core>

#include "types.h"

#include "geometry/camera/models.h" //UnifiedCameraModel

enum Mode {SO = 1, SE = 3, SIM = 4, AFF = 6};

struct Transformation
{
    Mode mode;
    int dimension; // TODO: gerer si on veux optimiser la transformation en 3D
                   // (pour l'instant elle ne fait que tu 2D)
    v::UnifiedCameraModel ucm;
    Eigen::Matrix<double, 3, 3> total_matrix;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Transformation(const Mode m = SO,
                   const int d = 2,
                   const v::UnifiedCameraModel& u = v::UnifiedCameraModel{},
                   const Eigen::Matrix<double, 3, 3>& tm = Eigen::Matrix<double, 3, 3>::Identity())
    : mode(m), dimension(d), ucm(u), total_matrix(tm)
    {};

    Eigen::Matrix<double, 3, 3> operator()() const;

    // TODO: dégager les calculs par zero
    void reset();

    // pinhole mode
    Eigen::Matrix<double, 3, 3> k() const;
};

/*
 * @Brief exponential_map
 * Compute the exponential map according to lie algebra rules
 * @range the lie generators
 * @x the state vector
**/
Eigen::Matrix<double, 3, 3> exponential_map(auto range, const Eigen::VectorXd& x);

//Using Eade lie algebra bases
const AlignedVector<Eigen::Matrix<double, 3, 3>>& lie_algebra_generators();

//Compute the exponential map of a matrix (according to the Lie algebra)
Eigen::Matrix<double, 3, 3> compute_exponential_map(const Eigen::VectorXd& x, int k);

Eigen::Matrix<double, 3, 3> get_exponential_map(const Transformation& t, const Eigen::VectorXd& a);

const Eigen::MatrixXd get_transformation_jacobian(const Transformation& t);
