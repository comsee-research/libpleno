#pragma once

#include <iostream>

#include "types.h"

inline void export_radii(const std::vector<std::vector<P2D>>& data)
{
	std::ofstream ofs("radii-"+std::to_string(getpid())+".csv");
	if (!ofs.good())
		throw std::runtime_error(std::string("Cannot open file errors.csv"));
	
	ofs << "type,invf,r\n";
	
	for(int type = 0; type < int(data.size()); ++type)
	{
		for(const P2D& d: data[type])
		{
			std::ostringstream oss;
			
			oss << type << ","
				<< d[0]<< ","
				<< d[1] << "\n"; 
			
			ofs << oss.str();
		}
	}
}
