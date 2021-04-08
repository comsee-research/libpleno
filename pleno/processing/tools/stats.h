#pragma once

#include <array>
#include <vector>
#include <numeric>

////////////////////////////////////////////////////////////////////////////////
// STD DEVIATION
////////////////////////////////////////////////////////////////////////////////
inline double variance(std::vector<double>& v)
{
	double mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0,
        [](double const & x, double const & y) { return x + y; },
        [mean](double const & x, double const & y) { return (x - mean)*(y - mean); });
    return sq_sum / (v.size() - 1 );
}

inline double stddev(std::vector<double> & v)
{
    return std::sqrt(variance(v));
}


////////////////////////////////////////////////////////////////////////////////
// SKEWNESS
////////////////////////////////////////////////////////////////////////////////
inline double skewness(std::vector<double> &v)
{
	double mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
	
    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0,
        [](double const & x, double const & y) { return x + y; },
        [mean](double const & x, double const & y) { return (x - mean)*(y - mean); });
	
    double cb_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0,
        [](double const & x, double const & y) { return x + y; },
        [mean](double const & x, double const & y) { return (x - mean)*(y - mean)*(x - mean); });
    
    double sigma = sq_sum / (v.size() - 1);
    
    return cb_sum / (v.size() * sigma * std::sqrt(sigma));
}

////////////////////////////////////////////////////////////////////////////////
// KURTOSIS
////////////////////////////////////////////////////////////////////////////////
inline double kurtosis(std::vector<double> &v)
{
	double mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
	
    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0,
        [](double const & x, double const & y) { return x + y; },
        [mean](double const & x, double const & y) { return (x - mean)*(y - mean); });
	
    double sq_sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0,
        [](double const & x, double const & y) { return x + y; },
        [mean](double const & x, double const & y) { return (x - mean)*(y - mean)*(x - mean)*(y - mean); });
    
    double sigma = sq_sum / (v.size() - 1);
    
    return sq_sq_sum / (v.size() * sigma * sigma);
}

////////////////////////////////////////////////////////////////////////////////
// INTERQUARTILES RANGE
////////////////////////////////////////////////////////////////////////////////
inline double iqr(std::vector<double> &v, double& q1, double &med, double &q3)
{
	auto const Q1 = v.size() / 4;
	auto const Q2 = v.size() / 2;
	auto const Q3 = Q1 + Q2;

	std::nth_element(v.begin(),          v.begin() + Q1, v.end());
	std::nth_element(v.begin() + Q1 + 1, v.begin() + Q2, v.end());
	std::nth_element(v.begin() + Q2 + 1, v.begin() + Q3, v.end());
	
	q1 = v[Q1];
	med = v[Q2];
	q3 = v[Q3];
	
	return q3-q1;
}

////////////////////////////////////////////////////////////////////////////////
// MEAN
////////////////////////////////////////////////////////////////////////////////
inline double mean(const std::vector<double>& v)
{
    return std::accumulate(begin(v), end(v), 0.0) / (double(size(v)) + 1e-3);
}

inline double mean(const std::array<double, 0>& /*a*/)
{
    return 0.0;
}

inline double mean(const std::array<double, 1>& a)
{
    return a[0];
}

template<typename Type, size_t Size>
inline Type mean(const std::array<Type, Size>& a)
{
    return std::accumulate(begin(a), end(a), 0.0) / (Type(Size) + 1e-3);
}


////////////////////////////////////////////////////////////////////////////////
// MEDIAN
////////////////////////////////////////////////////////////////////////////////
inline double median(std::vector<double> v)
{
    if (size(v) == 0)
        return 0.0;

    // using nth_element won't interpolate values for pair sized vectors
    std::nth_element(begin(v), begin(v) + size(v) / 2, end(v));
    return v[size(v) / 2];

}


inline double median(std::array<double, 0> /*a*/)
{
    return 0.0;
}

inline double median(std::array<double, 1> a)
{
    return a[0];
}

template<typename Type, size_t Size>
inline Type median(std::array<Type, Size> a)
{
    if (Size == 0)
    	return 0.0;

    // using nth_element won't interpolate values for pair sized vectors
    std::nth_element(begin(a), begin(a) + Size / 2, end(a));
    return a[Size / 2];
}
