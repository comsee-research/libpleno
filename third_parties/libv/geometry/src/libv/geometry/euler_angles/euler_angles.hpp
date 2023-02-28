/**

\file
Definition of the EulerAngles class.
\author Clement Deymier (2012)
\copyright 2012 Institut Pascal

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef LIBV_GEOMETRY_EULER_ANGLES_EULER_ANGLES_HPP
#define LIBV_GEOMETRY_EULER_ANGLES_EULER_ANGLES_HPP

#include <Eigen/Core>
#include <libv/geometry/global.hpp>

namespace v
{
  namespace geometry
  {

    /// \brief A structure containing euler angles for 3d rotations
    /// The class \em EulerAngles is 3-dimensional array for 3d geometry
    /// Angle are double by default for numerical stability
    class EulerAngles : public Eigen::Vector3d
    {

      public :

        typedef         double  		value_type;                          ///< The type of the array elements
        static const    std::size_t		nb_elements     = 3;                 ///< The array size (in elements) = 3
        static const    std::size_t 	size_in_bytes   = 3*sizeof(double);  ///< Total size of the array data (in bytes)

        /// \brief Class default constructor
        /// Creates an uninitialized EulerAngles.
        EulerAngles(){}

        /// \brief Class constructor
        /// Creates an EulerAngles from 3 value
        /// \param[in]  a    first value of EulerAngles - alpha
        /// \param[in]  b    second value of EulerAngles - beta
        /// \param[in]  g    third value of EulerAngles - gamma
        EulerAngles(value_type a,value_type b,value_type g)
        {
          (*this)[0] = a;
          (*this)[1] = b;
          (*this)[2] = g;
        }

        /// \brief Class copy constructor
        /// Creates a copy of the given array.
        /// \param[in]  ea   EulerAngles to copy the data from
        EulerAngles(const EulerAngles & ea)
          : Eigen::Vector3d(ea)
        {
          (*this)[0] = ea[0];
          (*this)[1] = ea[1];
          (*this)[2] = ea[2];
        }

        /// \brief \em EulerAngles data Accessor
        /// Gives an access to the first element of this EulerAngles
        /// \return A non-const reference to the first element of EulerAngles
        value_type & a(){return (*this)[0];}

        /// \brief \em EulerAngles data Accessor
        /// Gives an access to the second element of this EulerAngles
        /// \return A non-const reference to the second element of EulerAngles
        value_type & b(){return (*this)[1];}

        /// \brief \em EulerAngles data Accessor
        /// Gives an access to the third element of this EulerAngles
        /// \return A non-const reference to the third element of EulerAngles
        value_type & g(){return (*this)[2];}

        /// \brief \em EulerAngles data Accessor
        /// Gives an access to the first element of this EulerAngles
        /// \return A copy of the first element of EulerAngles
        value_type a()const{return (*this)[0];}

        /// \brief \em EulerAngles data Accessor
        /// Gives an access to the second element of this EulerAngles
        /// \return A copy of the second element of EulerAngles
        value_type b()const{return (*this)[1];}

        /// \brief \em EulerAngles data Accessor
        /// Gives an access to the third element of this EulerAngles
        /// \return A copy of the third element of EulerAngles
        value_type g()const{return (*this)[2];}

        /// \brief \em EulerAngles data modifier
        /// Gives an access to set the first element of this EulerAngles
        void a(value_type value){(*this)[0] = value;}

        /// \brief \em EulerAngles data modifier
        /// Gives an access to set the second element of this EulerAngles
        void b(value_type value){(*this)[1] = value;}

        /// \brief \em EulerAngles data modifier
        /// Gives an access to set the third element of this EulerAngles
        void g(value_type value){(*this)[2] = value;}

    };

  }

}

#endif
