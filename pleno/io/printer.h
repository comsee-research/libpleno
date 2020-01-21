#pragma once

#include <iostream>

class Printer : std::iostream, std::streambuf
{
public:
	enum Level : std::uint16_t {NONE= 0x00, ERR= (0x01 << 0), WARN=(0x01 << 1), INFO= (0x01 << 2), DEBUG= (0x01 << 3), ALL= ERR|WARN|INFO|DEBUG};
private:
	static Printer _self;
	static bool _verbose;
	static std::uint16_t _level;
	
	Printer() {};
public:
	static bool verbose() {return _verbose;}
	static void verbose(bool v) {_verbose=v;}
	static std::uint16_t level() {return _level;}
	static void level(std::uint16_t l) {_level=l;}
	
	static std::ostream& err(){ if(_verbose and (_level bitand ERR)) return std::cerr; return _self;} 
	static std::ostream& warn(){ if(_verbose and (_level bitand WARN)) return std::cout; return _self;} 
	static std::ostream& info(){ if(_verbose and (_level bitand INFO)) return std::cout; return _self;} 
	static std::ostream& debug(){ if(_verbose and (_level bitand DEBUG)) return std::cout; return _self;} 
};

#define PRINT_ERR(...) (Printer::err() << "\033[1;31mERROR:\033[0m " << __VA_ARGS__ << std::endl) /* colored in red */
#define PRINT_WARN(...) (Printer::warn() << "\033[4;33mWARNING:\033[0m " << __VA_ARGS__ << std::endl) /* colored and underlined in yellow */
#define PRINT_INFO(...) (Printer::info() << "\033[32mINFO:\033[0m " << __VA_ARGS__ << std::endl) /* colored in green */
#define PRINT_DEBUG(...) (Printer::debug() << "\033[34mDEBUG:\033[0m " << __VA_ARGS__ << std::endl) /* colored in blue */

#define DEBUG_ASSERT(Condition, ...) do { if((not (Condition))) { Printer::verbose(true); Printer::level(Printer::Level::ALL); (PRINT_ERR(__VA_ARGS__)); assert((Condition)); abort(); } } while (0)

#define	DEBUG_TO_STR(VAR) #VAR
#define DEBUG_VAR(VAR)	PRINT_DEBUG(DEBUG_TO_STR(VAR) <<"= " << VAR)
