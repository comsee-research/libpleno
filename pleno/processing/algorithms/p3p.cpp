#include "p3p.h"

#include "geometry/pose.h"

/**

Solve the quartic equation \f$ ax^4+bx^3+cx^2+dx+e=0 \f$.

Use <a href="http://en.wikipedia.org/wiki/Quartic_function#Ferrari.27s_solution">Ferrari’s closed form solution</a>.

\return
A list of 4 real solutions.

*/
Eigen::Vector4d solve_quartic
( double A ///< The coefficient of degree 4
, double B ///< The coefficient of degree 3
, double C ///< The coefficient of degree 2
, double D ///< The coefficient of degree 1
, double E ///< The coefficient of degree 0
)
{
  double alpha = -3 * B*B / (8 * A*A) + C / A;
  double beta = B*B*B / (8 * A*A*A) - B * C / (2 * A*A) + D / A;
  double gamma = -3 * B*B*B*B / (256 * A*A*A*A) + B*B * C / (16 * A*A*A) - B * D / (4 * A*A) + E / A;

  std::complex<double> P = -alpha*alpha / 12 - gamma;
  std::complex<double> Q = -alpha*alpha*alpha / 108 + alpha * gamma / 3 - beta*beta / 8;
  std::complex<double> R = -Q / 2. + std::sqrt(Q*Q / 4. + P*P*P / 27.);
  std::complex<double> U = std::pow(R, 1. / 3.);
  std::complex<double> y = U.real()
    ? -5 * alpha / 6 - P / (3. * U) + U
    : -5 * alpha / 6 - std::pow(Q, 1. / 3.);
  std::complex<double> w = std::sqrt(alpha + 2. * y);

  Eigen::Vector4d roots;
  roots <<
    (-B / A / 4 + (+w + std::sqrt(-(3 * alpha + 2. * y + 2 * beta / w))) / 2.).real(),
    (-B / A / 4 + (+w - std::sqrt(-(3 * alpha + 2. * y + 2 * beta / w))) / 2.).real(),
    (-B / A / 4 + (-w + std::sqrt(-(3 * alpha + 2. * y - 2 * beta / w))) / 2.).real(),
    (-B / A / 4 + (-w - std::sqrt(-(3 * alpha + 2. * y - 2 * beta / w))) / 2.).real();
  return roots;
}

/**

Solve the P3P problem.

Compute the absolute pose of a camera using three 3D-to-2D correspondences.

The code was shamelessly stolen from \cite Kneip5995464.
Notations and comments marked with a • are the same as in \cite Kneip5995464.

\return
- True if solutions were found;
- False if the points are aligned.

\author Laurent Kneip (Nov 2, 2010)
\copyright 2011, Laurent Kneip, ETH Zurich.
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
- Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
- Neither the name of ETH Zurich nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
bool solve_p3p( 
	Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector3d P3, // A point in the world reference frame
   	Eigen::Vector3d f1, // The direction of a ray out of the camera and pointing towards \e P1
  	Eigen::Vector3d f2, // Idem for \e P2
   	Eigen::Vector3d _f3, // Idem for \e P3
   	Poses& solutions // An array of 4 possible camera poses
)
{
  solutions.clear();
  solutions.resize(4);
  
  // bail out if P1, P2, P3 are colinear
  if(((P2 - P1).cross(P3 - P1)).squaredNorm() == 0) return false;

  // • compute the transformation matrix T and the feature vector f3 using (1)
  Eigen::Matrix3d T;
  T.row(0) = f1;
  T.row(2) = f1.cross(f2).normalized();
  T.row(1) = T.row(2).cross(T.row(0));
  Eigen::Vector3d f3 = T * _f3;

  // make sure that θ ∈ [0;π]
  if(f3.z() > 0)
  {
    f1.swap(f2);
    P1.swap(P2);

    T.row(0) = f1;
    T.row(2) = f1.cross(f2).normalized();
    T.row(1) = T.row(2).cross(T.row(0));
    f3 = T * _f3;
  }

  // • compute the transformation matrix N and the world point P3 using (2)
  Eigen::Matrix3d N;
  N.row(0) = (P2 - P1).normalized();
  N.row(2) = (P2 - P1).cross(P3 - P1).normalized();
  N.row(1) = N.row(2).cross(N.row(0));
  P3 = N * (P3 - P1);

  // • extract p1 and p2 from P3
  double p1 = P3.x();
  double p2 = P3.y();

  // • compute d12 and b using (3)
  double d12 = (P2 - P1).norm();
  double cos_beta = f1.transpose() * f2;
  double b = std::sqrt(1 / (1 - cos_beta*cos_beta) - 1) * (cos_beta < 0 ? -1 : 1);

  // • compute φ1 and φ2 using (8)
  double phi1 = f3.x() / f3.z();
  double phi2 = f3.y() / f3.z();

  // • compute the factors a4, a3, a2, a1, and a0 of polynomial (11)
  // • find the real roots of the polynomial (values for cos θ)
  Eigen::Vector4d roots = solve_quartic
    (
    - p2*p2*p2*p2 * phi2*phi2
    - p2*p2*p2*p2 * phi1*phi1
    - p2*p2*p2*p2
    ,
    + 2 * p2*p2*p2 * d12 * b
    + 2 * phi2*phi2 * p2*p2*p2 * d12 * b
    - 2 * phi2 * p2*p2*p2 * phi1 * d12
    ,
    - phi2*phi2 * p2*p2 * p1*p1
    - phi2*phi2 * p2*p2 * d12*d12 * b*b
    - phi2*phi2 * p2*p2 * d12*d12
    + phi2*phi2 * p2*p2*p2*p2
    + p2*p2*p2*p2 * phi1*phi1
    + 2 * p1 * p2*p2 * d12
    + 2 * phi1 * phi2 * p1 * p2*p2 * d12 * b
    - p2*p2 * p1*p1 * phi1*phi1
    + 2 * p1 * p2*p2 * phi2*phi2 * d12
    - p2*p2 * d12*d12 * b*b
    - 2 * p1*p1 * p2*p2
    ,
    + 2 * p1*p1 * p2 * d12 * b
    + 2 * phi2 * p2*p2*p2 * phi1 * d12
    - 2 * phi2*phi2 * p2*p2*p2 * d12 * b
    - 2 * p1 * p2 * d12*d12 * b
    ,
    - 2 * phi2 * p2*p2 * phi1 * p1 * d12 * b
    + phi2*phi2 * p2*p2 * d12*d12
    + 2 * p1*p1*p1 * d12
    - p1*p1 * d12*d12
    + phi2*phi2 * p2*p2 * p1*p1
    - p1*p1*p1*p1
    - 2 * phi2*phi2 * p2*p2 * p1 * d12
    + p2*p2 * phi1*phi1 * p1*p1
    + phi2*phi2 * p2*p2 * d12*d12 * b*b
    );

  for(int i = 0; i < 4; ++i)
  {
    double cos_theta = roots[i];

    // • for each solution, find the values for cot α using (9)
    double cot_alpha = (-phi1 * p1 / phi2 - cos_theta * p2 + d12 * b) / (-phi1 * cos_theta * p2 / phi2 + p1 - d12);

    // • compute all necessary trigonometric forms of α and θ using trigonometric relationships and the restricted parameter domains
    double sin_theta = cos_theta*cos_theta < 1 ? std::sqrt(1 - cos_theta*cos_theta) : 0;
    double sin_alpha = std::sqrt(1 / (cot_alpha*cot_alpha + 1));
    double cos_alpha = std::sqrt(1 - sin_alpha*sin_alpha) * (cot_alpha < 0 ? -1 : 1);

    // • for each solution, compute C η and Q using (5) and (6), respectively
    Eigen::Vector3d C; C <<
      d12 * cos_alpha * (sin_alpha * b + cos_alpha),
      d12 * sin_alpha * (sin_alpha * b + cos_alpha) * cos_theta,
      d12 * sin_alpha * (sin_alpha * b + cos_alpha) * sin_theta;
    Eigen::Matrix3d R; R <<
      -cos_alpha, -sin_alpha*cos_theta, -sin_alpha*sin_theta,
      +sin_alpha, -cos_alpha*cos_theta, -cos_alpha*sin_theta,
      0,          -sin_theta,           +cos_theta;

    // • for each solution, compute the absolute camera center C and orientation R using (12) and (13), respectively
    solutions[i].translation() = (P1 + N.transpose() * C);
    solutions[i].rotation() = (N.transpose() * R.transpose() * T);
  }

  return true;
}

