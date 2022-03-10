// Fluid3D.cpp : Defines the entry point for the application.
//

#define _USE_MATH_DEFINES

#include "fluid3D.h"
#include "simulation.h"
#include "event_driven_collisions.h"

#include <memory>
#include <vector>
#include <random>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <string>

using std::filesystem::current_path;
using namespace Fluid3D_namespace;

void init_timestep_simulation(Simulation & sim);

Fluid3D::Fluid3D()
{
	//will eventualy setup inputs, external libraries and any other dependencies
}

void Fluid3D::save_to_file(const Simulation &sim)
{
	std::ofstream f;
	std::filesystem::path filePath = std::filesystem::current_path();
	filePath.append("fluid_data.csv");
	f.open(filePath.string());
	f << "sim_size," << "box_size," << "collisions," << "timesteps" << "\n";
	f << sim._particles << "," << sim._box_size << "," << sim._collisions << "," << (sim._run_time / sim._time_save_interval) + 1 << "\n";
	f << "Timestep";
	for (int j = 0; j < sim._particles; j += 1)
	{
		f << ",x" << j << ",y" << j << ",z" << j << ",vx" << j << ",vy" << j << ",vz" << j;
	}
	f << "\n";

	for (int it_row = 0; it_row < (sim._run_time / sim._time_save_interval) + 1; it_row++)
	{
		f << it_row;
		for (int it_col = 0; it_col < (sim._particles * 6); it_col++)
		{
			f << "," << sim._particle_data[it_col + sim._particles * 6 * it_row];
		}
		f << "\n";
	}
	f.close();
}


int main()
{
	Fluid3D fluid3D;
	Simulation sim;
	setup_computation(sim);
	//init_timestep_simulation(sim);
	fluid3D.save_to_file(sim);
	return 0;
}