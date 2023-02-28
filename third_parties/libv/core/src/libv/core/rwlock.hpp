/**

\file
\author Stéphane Witzmann (2014)
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

#ifndef LIBV_VBUS_TOOLS_RWLOCK_HPP
#define LIBV_VBUS_TOOLS_RWLOCK_HPP

#include <mutex>
#include <condition_variable>

#include <libv/core/noncopyable.hpp>
#include <libv/core/global.hpp>

namespace v {
namespace core {

/*
 * Reader-Writer lock (multiple readers / single writer)
 * See: http://en.wikipedia.org/wiki/Readers%E2%80%93writer_lock
 *
 * Our implementation favors writers over readers.
 * Also, we (currently) don't need upgradable readers.
 */
class LIBV_CORE_EXPORT RwLock: private v::core::noncopyable
{
public:
  /*
   * Reader
   */
  class Reader: private v::core::noncopyable
  {
  public:
    explicit Reader(RwLock &);
    ~Reader();

  private:
    RwLock &parent_;
  };

  /*
   * Writer
   */
  class Writer: private v::core::noncopyable
  {
  public:
    explicit Writer(RwLock &);
    ~Writer();

  private:
    RwLock &parent_;
  };

  RwLock();
  ~RwLock();

private:
  std::mutex mtx_;
  std::condition_variable rcnd_;
  std::condition_variable wcnd_;
  unsigned rwait_;
  unsigned ractive_;
  unsigned wwait_;
  unsigned wactive_;

  friend class Reader;
  friend class Writer;
};

}
}

#endif
