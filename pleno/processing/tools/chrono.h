#pragma once

#include <chrono>

class Chrono 
{
	using Time_t = decltype(std::chrono::high_resolution_clock::now());
	
	static Time_t start;
	static Time_t stop;
	
	static double time;
	
	Chrono() {};
public:
	static void tic() { start = std::chrono::high_resolution_clock::now(); }
	static void tac() { stop = std::chrono::high_resolution_clock::now(); time = std::chrono::duration<double>(stop-start).count(); }
	
	static double get() { return time; }
};

