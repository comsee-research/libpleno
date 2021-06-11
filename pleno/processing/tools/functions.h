#pragma once

struct LinearFunction {
	double a = 1., b = 0.;
	
	double operator()(double x) const { return a * x + b; }
	double apply(double x) const { return this->operator()(x); }
	
	LinearFunction inverse() const { return LinearFunction{1. / a, - b / a}; }
};

struct QuadraticFunction {
	double a = 0., b = 1., c = 0.;
	
	double operator()(double x) const { return a * x * x + b * x + c; }
	double apply(double x) const { return this->operator()(x); }
};

struct CubicFunction {
	double a = 0., b = 0., c = 1., d = 0.;
	
	double operator()(double x) const { return a * x * x * x + b * x * x + c * x + d; }
	double apply(double x) const { return this->operator()(x); }
};

struct QuarticFunction {
	double a = 0., b = 0., c = 0., d = 1., e = 0.;
	
	double operator()(double x) const { return a * x * x * x * x + b * x * x * x + c * x * x + d * x + e; }
	double apply(double x) const { return this->operator()(x); }
};
