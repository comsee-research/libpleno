#include "chrono.h"

double Chrono::time = 0.;

Chrono::Time_t Chrono::start = std::chrono::high_resolution_clock::now();
Chrono::Time_t Chrono::stop = Chrono::start;
