/**

\file
Load and save images.
\author Alexis Wilhelm (2013)
\copyright 2013 Institut Pascal

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

#include <libv/core/auto_load.hpp>
#include <libv/core/image/image.hpp>
#include <libv/core/image/io.hpp>

namespace v {
namespace core {

/**

Load an image.

\param input An image file.
\param output An image in memory.
  Will be resized if necessary.

*/
LIBV_CORE_EXPORT void
load(const std::string &input, v::ImageU8 &output)
{
  auto_load(input, output);
}

/// \copydoc load
LIBV_CORE_EXPORT void
load(const std::string &input, v::ImageRGBU8 &output)
{
  auto_load(input, output);
}

/// \copydoc load
LIBV_CORE_EXPORT void
load(const std::string &input, v::ImageRGBAU8 &output)
{
  auto_load(input, output);
}

/**

Save an image.

\param input An image.
\param output A file name.

*/
LIBV_CORE_EXPORT void
save(const std::string &output, const v::ImageU8cr &input)
{
  auto_load(input, output);
}

/// \copydoc save
LIBV_CORE_EXPORT void
save(const std::string &output, const v::ImageRGBU8cr &input)
{
  auto_load(input, output);
}

/// \copydoc save
LIBV_CORE_EXPORT void
save(const std::string &output, const v::ImageRGBAU8cr &input)
{
  auto_load(input, output);
}

}}
