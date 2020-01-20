#include "printer.h"

bool Printer::_verbose = true;
std::uint16_t Printer::_level = Printer::Level::ALL;

Printer Printer::_self{};

