/**

\file
\author Alexis Wilhelm (2015)
\copyright 2015 Institut Pascal

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

#include <chrono>
#include <cstring>
#include <iostream>
#include <map>

#include "logger.hpp"

namespace v {
namespace core {
namespace {

using Time = std::chrono::steady_clock;

struct Tuple
{
  size_t count = 0;
  Time::duration max{0};
  Time::duration total{0};
};

static float to_float(const Time::duration &d)
{
  return float(d.count()) * Time::period::num / Time::period::den;
}

struct Data
: std::map<const char *, Tuple>
{
  ~Data()
  {
    for(auto &&p: *this)
    {
      const float d = to_float(p.second.total);
      Logger<true>{v::log_stderr, p.first}
        << " calls=" << p.second.count
        << " mean=" << d / p.second.count
        << " max=" << to_float(p.second.max)
        << " total=" << d
        ;
    }
  }
};

template<std::ostream &_stream>
static void log_ostream(std::ostringstream *s)
{
  // idéalement, il faudrait verrouiller un mutex ici, mais alors on introduirait une dépendance à une bibliothèque de threads
  // static std::mutex mutex; std::lock_guard<std::mutex> lock(mutex);
  _stream << s->str();
}

}

void log_stdout(std::ostringstream *s) { return log_ostream<std::cout>(s); }
void log_stderr(std::ostringstream *s) { return log_ostream<std::cerr>(s); }

/**

Raccourcit un chemin pour alléger les messages de debug.

Quand on préfixe les messages de debug par le chemin vers le fichier d'où ils viennent, ça peut faire un affichage très lourd si le chemin est un peu long.
Pour alléger l'affichage, on raccourcit ce chemin en ne gardant que le nom du fichier.

*/
const char *shorten_file_name(const char *full)
{
  const char *shortened = std::strrchr(full, '/');
  return shortened ? shortened + 1 : full;
}

/*******************************************************************//**

\class Profiler

Mesure le temps d'exécution d'une portion de programme.

Cette classe ne doit pas être utilisée directement.
Utilisez la macro #V_PROFILER à la place.

\def V_PROFILER

Instancie un \ref v::core::Profiler "Profiler" pour mesurer le temps d'exécution d'une portion de programme.

Les temps mesurés et les statistiques sont affichées quand le programme se termine.

Par exemple, insérer cette ligne dans src/file.cpp :
\code
  V_PROFILER(my_profiler);
\endcode
affichera, à la fin du programme :
\code
  src/file.cpp:99: my_profiler: calls=8 mean=0.000113148 max=0.000623207 total=0.000905188
\endcode

Cette macro instancie une variable `var`.
On commence à chronométrer dès que cette variable est instanciée, et on arrête quand elle est détruite (elle est détruite automatiquement quand elle devient hors portée).

Dans certains cas, on peut vouloir arrêter de chronométrer manuellement : dans ce cas on utilise `var.stop()`.
Pour recommencer à chronométrer, on peut utiliser `var.start()`.

\param var
Le nom de la variable qui sera instanciée.
On pourra utiliser cette variable pour appeler les fonctions stop() et start() si nécessaire.

***********************************************************************/

/**

Initialise un profiler et commence à chronométrer.

*/
Profiler::Profiler(const char *id)
: id_{id}
, begin_{0}
{
  start();
}

/**

Arrête de chronométrer quand ce profiler est détruit.

*/
Profiler::~Profiler()
{
  if(begin_)
  {
    stop();
  }
}

/**

Recommence à chronométrer.

*/
void Profiler::start()
{
  begin_ = Time::now().time_since_epoch().count();
}

/**

Arrête de chronométrer.

*/
void Profiler::stop()
{
  const Time::duration d = Time::now() - Time::time_point{Time::duration{begin_}};
  static Data data;
  Tuple &p = data[id_];
  ++p.count;
  p.total += d;
  if(d > p.max) p.max = d;
  begin_ = 0;
}

}
}
