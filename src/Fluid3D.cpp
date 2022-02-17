// Fluid3D.cpp : Defines the entry point for the application.
//

#define _USE_MATH_DEFINES


#include "fluid3D.h"
#include "laser.h"
#include "simulation.h"

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

std::tuple<std::vector<double>, std::vector<double>, int> workhorse(Simulation sim);

Fluid3D::Fluid3D()
{
}

void Fluid3D::save_to_file(Simulation sim, std::tuple<std::vector<double>, std::vector<double>, int> data)
{
	auto positional_data = std::get<0>(data);
	auto velocity_data = std::get<1>(data);
	auto collisions = std::get<2>(data);
	std::ofstream f;
	std::filesystem::path filePath = current_path();
	filePath.append("fluid_data.csv");
	f.open(filePath.string());
	f << "sim_size," << "box_size," << "collisions," << "timesteps" << "\n";
	f << sim.particle_count << "," << sim.d.material_type.width << "," << collisions << "," << sim.steps << "\n";
	f << "Timestep";
	for (int j = 0; j < sim.particle_count; j += 1)
	{
		f << ",x" << j << ",y" << j << ",z" << j << ",vx" << j << ",vy" << j << ",vz" << j;
	}
	f << "\n";

	for (int i = 0; i < sim.steps; i++)
	{
		f << i;
		for (int j = 0; j < (sim.particle_count*3); j += 3)
		{
			f << "," << positional_data[j + sim.particle_count * 3 * i] << "," << positional_data[j + sim.particle_count * 3 * i + 1] << "," << positional_data[j + sim.particle_count * 3 * i + 2];
			f << "," << velocity_data[j + sim.particle_count * 3 * i] << "," << velocity_data[j + sim.particle_count * 3 * i + 1] << "," << velocity_data[j + sim.particle_count * 3 * i + 2];
			//f << "," << acceleration_data[j + sim_size * i] << "," << acceleration_data[j + sim_size * i + 1] << "," << acceleration_data[j + sim_size * i + 2];
		}
		f << "\n";
	}
	f.close();
}

int main()
{
	Fluid3D p;
	Simulation sim;
	std::tuple<std::vector<double>, std::vector<double>, int> data = workhorse(sim);
	p.save_to_file(sim, data);
	return 0;
}