#pragma once
#include <vector>
#include <random>

class Random_Setup
{
public:
	std::uniform_real_distribution<double> _pos_range;
	std::uniform_real_distribution<double> _vel_range;
	Random_Setup(double maxv, double minv, double maxp, double minp);
};