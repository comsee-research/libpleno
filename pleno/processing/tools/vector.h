#pragma once

#include <vector>
#include <algorithm>
#include <iterator>
#include <random>

#define SAME_SEED 1
template<class BidirectIterator>
BidirectIterator random_n_unique(BidirectIterator begin, BidirectIterator end, std::size_t num_random) 
{
#if SAME_SEED 
    static thread_local std::mt19937 mt;
#else
    static thread_local std::random_device rd;
    static thread_local std::mt19937 mt(rd());
#endif
    std::size_t left = std::distance(begin, end)-1;
    while (num_random--) 
    {
    	std::uniform_int_distribution<> dis(0, left);
    	
        BidirectIterator r = begin;
        std::advance(r, dis(mt));
        std::iter_swap(begin, r);
        ++begin;
        --left;
    }
    return begin;
}
