#pragma once

#include <iostream>
#include <string>

#include <pleno/graphic/gui.h>
#include <pleno/io/printer.h>

inline bool yes_no_question(std::string msg) 
{
	bool ret = false;
	if(Printer::level() bitand Printer::Level::WARN)
	{
		PRINT_WARN(msg << "? [y/n]");
		char c;
		std::cin >> c;
		if(c == 'y') { ret = true; }	
		std::cin.clear();
		while (std::cin.get() != '\n');
	}
	return ret;
}

inline bool finished() 
{
	return yes_no_question("Finished");
}

inline bool save() 
{
	return yes_no_question("Save");
}

inline void clear() 
{
	GUI(
		PRINT_WARN("Clear viewer ? [y/n]");
		char c;
		std::cin >> c;
		if(c == 'y') {Viewer::clear(); PRINT_DEBUG("Cleared !"); }	
		std::cin.clear();
		while (std::cin.get() != '\n');
	);
}

inline void wait() 
{
	if(Printer::level() bitand Printer::Level::WARN)
	{
		PRINT_WARN("Press key to continue...");
		std::getchar();
	}
}


