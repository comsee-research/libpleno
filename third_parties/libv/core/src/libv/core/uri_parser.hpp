/**

\file
\author Alexis Wilhelm (2014)
\copyright 2014 Institut Pascal

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

#ifndef LIBV_CORE_URI_PARSER_HPP
#define LIBV_CORE_URI_PARSER_HPP

#include <libv/core/global.hpp>
#include <boost/lexical_cast.hpp>
#include <regex>

namespace v {
namespace core {
/// \addtogroup io
/// \{

/**

Une expression régulière pour analyser des URI.
Utile pour identifier des capteurs ou d'autres ressources à ouvrir avec auto_load().

*/
struct LIBV_CORE_EXPORT UriParser
{
  /**

  Initialise avec un motif.

  */
  UriParser
  ( const std::string &pattern ///< Le motif auquel les URI devront se conformer.
  )
  : pattern_(pattern)
  , regex_(pattern, std::regex_constants::optimize)
  {
  }

  /**

  Analyse un URI.

  \return
  Le contenu des masques capturants.

  \throw std::invalid_argument
  L'URI analysé n'est pas conforme au motif.

  */
  std::smatch match
  ( const std::string &uri ///< L'URI à analyser.
  ) const
  {
    std::smatch matches;
    if(regex_match(uri, matches, regex_)) return matches;
    throw std::invalid_argument(uri + " does not match " + pattern_);
  }

private:

  std::string pattern_;
  std::regex regex_;
};

/// \}
}
}

#endif
