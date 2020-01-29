#include "printer.h"

bool Printer::verbose_ = true;
std::uint16_t Printer::level_ = Printer::Level::ALL;

Printer Printer::self_{};

