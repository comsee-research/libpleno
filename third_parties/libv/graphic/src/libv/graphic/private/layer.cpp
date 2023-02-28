/**

\file
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

#include "layer.hpp"

#include <iostream>

namespace v {
namespace graphic {
namespace viewers_ {

#if defined NDEBUG
#define _CHECK(x)
#else
#define _CHECK(x) if(x) return warning();

/**

Le rayon maximal de la boîte englobante utilisée pour le zoom automatique.

Si on essaye d'afficher quelque chose au-delà de ce rayon, c'est sûrement qu'il y a un problème dans l'algorithme qui produit les données, donc on affiche un message dans la console.
On peut aussi mettre un point d'arrêt sur warning() pour trouver la source du problème.

Quand la boîte est vraiment trop grande, le zoom automatique ne fonctionne pas, donc on l'empêche de trop grandir.
Pour voir les objets au-delà de ce rayon, il faudra zoomer manuellement.

*/
static const qreal MAX = 1e9;

static void warning()
{
  std::cerr << "v::graphic::viewers_::warning(): this layer's bounding box has become very large." << std::endl;
}

#endif

Layer::Layer(void)
  : visible(true)
{
}

void
Layer::clear(void)
{
  data.clear();
  bounds = QRectF();
}

void
Layer::grow(const QPointF &p)
{
  _CHECK(p.x() < -MAX || p.x() > MAX || p.y() < -MAX || p.y() > MAX)
  if(p.x() < bounds.left()) bounds.setLeft(p.x());
  if(p.x() > bounds.right()) bounds.setRight(p.x());
  if(p.y() < bounds.top()) bounds.setTop(p.y());
  if(p.y() > bounds.bottom()) bounds.setBottom(p.y());
}

void
Layer::grow(const QRectF &a)
{
  _CHECK(a.left() < -MAX || a.right() > MAX || a.top() < -MAX || a.bottom() > MAX)
  bounds |= a;
}

}}}
