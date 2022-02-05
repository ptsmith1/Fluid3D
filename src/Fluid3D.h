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
		std::vector<double> x_pos;
		std::vector<double> y_pos;
		std::vector<double> z_pos;

		std::vector<double> x_vel;
		std::vector<double> y_vel;
		std::vector<double> z_vel;

		std::vector<double> x_acc;
		std::vector<double> y_acc;
		std::vector<double> z_acc;

		std::vector<int> charges;

		int box_size_angs;
		double box_size;
		int sim_size;
		int collisions;

		std::vector<double> positional_data;
		std::vector<double> velocity_data;
		std::vector<double> acceleration_data;
	public:
		int dt = 0;
		Fluid3D(const int total_steps);
		std::vector<double> get_x();
		std::vector<double> get_y();
		std::vector<double> get_z();
		void update_potentials(int step);
		void coulomb_acceleration();
		void update_velocities(int step);
		void update_positions(int step);
		void hits_box();
		void save_to_file(const int total_steps);
	};
}

// TODO: Reference additional headers your program requires here.