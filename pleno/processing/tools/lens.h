#pragma once

#include <cmath>

//WARNING: k,l are expressed in MIA space
inline int lens_type(std::size_t I, std::size_t k, std::size_t l)
{
	if(I == 0u) return -1;
	if(I == 1u) return 0;
	
	return static_cast<int>(std::fmod(std::fmod(l,(I-1))+k, I));
}
