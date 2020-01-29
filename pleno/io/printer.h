#pragma once

#include <iostream>
#include <cassert>


class Printer : std::iostream, std::streambuf
{
public:
	enum Level : std::uint16_t {NONE= 0x00, ERR= (0x01 << 0), WARN=(0x01 << 1), INFO= (0x01 << 2), DEBUG= (0x01 << 3), ALL= ERR|WARN|INFO|DEBUG};
private:
	static Printer self_;
	static bool verbose_;
	static std::uint16_t level_;
	
	Printer() {};
public:
	static bool verbose() {return verbose_;}
	static void verbose(bool v) {verbose_=v;}
	static std::uint16_t level() {return level_;}
	static void level(std::uint16_t l) {level_=l;}
	
	static std::ostream& err(){ if(verbose_ and (level_ bitand ERR)) return std::cerr; return self_;} 
	static std::ostream& warn(){ if(verbose_ and (level_ bitand WARN)) return std::cout; return self_;} 
	static std::ostream& info(){ if(verbose_ and (level_ bitand INFO)) return std::cout; return self_;} 
	static std::ostream& debug(){ if(verbose_ and (level_ bitand DEBUG)) return std::cout; return self_;} 
};

#define PRINT_ERR(...) (Printer::err() << "\033[1;31mERROR:\033[0m " << __VA_ARGS__ << std::endl) /* colored in red */
#define PRINT_WARN(...) (Printer::warn() << "\033[4;33mWARNING:\033[0m " << __VA_ARGS__ << std::endl) /* colored and underlined in yellow */
#define PRINT_INFO(...) (Printer::info() << "\033[32mINFO:\033[0m " << __VA_ARGS__ << std::endl) /* colored in green */
#define PRINT_DEBUG(...) (Printer::debug() << "\033[34mDEBUG:\033[0m " << __VA_ARGS__ << std::endl) /* colored in blue */

#define DEBUG_ASSERT(Condition, ...) do { if((not (Condition))) { Printer::verbose(true); Printer::level(Printer::Level::ALL); (PRINT_ERR(__VA_ARGS__)); assert((Condition)); abort(); } } while (0)

#define	DEBUG_TO_STR(VAR) #VAR
#define DEBUG_VAR(VAR)	PRINT_DEBUG(DEBUG_TO_STR(VAR) <<"= " << VAR)
