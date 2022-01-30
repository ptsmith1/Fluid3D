// Fluid3D.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <iostream>
#include <vector>

namespace Fluid3D_namespace
{
	class Fluid3D
	{
	private:
		std::vector<int> x_pos;
		std::vector<int> y_pos;
		std::vector<int> z_pos;

		std::vector<int> x_vel;
		std::vector<int> y_vel;
		std::vector<int> z_vel;

		int box_size;
		int sim_size;
		int collisions;

		std::vector<int> positional_data;
		std::vector<int> velocity_data;
	public:
		int dt;
		Fluid3D(const int total_steps);
		std::vector<int> get_x();
		std::vector<int> get_y();
		std::vector<int> get_z();
		void update_positions(int step);
		void hits_box();
		void save_to_file(const int total_steps);
	};
}

// TODO: Reference additional headers your program requires here.
