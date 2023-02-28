/**

\file
\author Stéphane Witzmann (2017)
\copyright 2017 Institut Pascal

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

#ifndef LIBV_CORE_MEMORY_ALIGNED_VECTOR_HPP
#define LIBV_CORE_MEMORY_ALIGNED_VECTOR_HPP

#include <libv/core/global.hpp>

#include <vector>
#include <Eigen/Core>

namespace v {
namespace core {

template<class T> using AlignedVector = std::vector<T,Eigen::aligned_allocator<T>>;

}}

#endif
