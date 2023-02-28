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

#include <cassert>

#include <libv/core/rwlock.hpp>

using namespace v::core;

#define LOCK(instance) std::lock_guard<std::mutex> lock(instance.mtx_);
#define ULOCK(instance) std::unique_lock<std::mutex> lock(instance.mtx_);

RwLock::RwLock():
  mtx_(),
  rcnd_(),
  wcnd_(),
  rwait_(0),
  ractive_(0),
  wwait_(0),
  wactive_(0)
{
}

RwLock::~RwLock()
{
  LOCK((*this))
  if (rwait_ > 0 || ractive_ > 0 || wwait_ > 0 || wactive_ > 0)
  {
    assert(false);
  }
}

RwLock::Reader::Reader(RwLock &parent):
  parent_(parent)
{
  ULOCK(parent_)
  ++parent_.rwait_;

  while (parent_.wwait_ > 0 || parent_.wactive_ > 0)
    parent_.rcnd_.wait(lock);

  --parent_.rwait_;
  ++parent_.ractive_;
}

RwLock::Reader::~Reader()
{
  LOCK(parent_)
  --parent_.ractive_;

  if (parent_.wwait_ > 0 && parent_.ractive_ == 0)
    parent_.wcnd_.notify_one();
}

RwLock::Writer::Writer(RwLock &parent):
  parent_(parent)
{
  ULOCK(parent_)
  ++parent_.wwait_;

  while (parent_.ractive_ > 0 || parent_.wactive_ > 0)
    parent_.wcnd_.wait(lock);

  --parent_.wwait_;
  ++parent_.wactive_;
}

RwLock::Writer::~Writer()
{
  LOCK(parent_)
  --parent_.wactive_;

  if (parent_.wwait_ > 0)
    parent_.wcnd_.notify_one();
  else if (parent_.rwait_ > 0)
    parent_.rcnd_.notify_all();
}

