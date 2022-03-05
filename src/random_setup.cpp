#include "random_setup.h"
#include <iostream>

Random_Setup::Random_Setup(double maxv, double minv, double maxp, double minp)
{
	_pos_range = std::uniform_real_distribution<double>{ minp, maxp };
	_vel_range = std::uniform_real_distribution<double>{ minv, maxv };
}
