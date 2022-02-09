// Fluid3D.cpp : Defines the entry point for the application.
//

#define _USE_MATH_DEFINES
#define magnetic_constant (1.6e-19*1.6e-19)/(4*M_PI*8.854e-12*9.10956e-31) 

#include "fluid3D.h"
#include "laser.h"
#include "simulation.cpp"

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

void random_vec_initialisation(std::vector<int>& vel);

int workhorse(Simulation sim);

Fluid3D::Fluid3D(int total_steps)
{
	const double electron_mass = 9.10956e-31;
	dt = 1;
	double particle_radius = 1e-10;
	box_size_angs = 100; //in angstroms
	box_size = box_size_angs * 1e-10;

	sim_size = 100;
	x_pos = std::vector<double>(sim_size);
	y_pos = std::vector<double>(sim_size);
	z_pos = std::vector<double>(sim_size);
	x_vel = std::vector<double>(sim_size);
	y_vel = std::vector<double>(sim_size);
	z_vel = std::vector<double>(sim_size);
	x_acc = std::vector<double>(sim_size);
	y_acc = std::vector<double>(sim_size);
	z_acc = std::vector<double>(sim_size);

	charges = std::vector<int>(sim_size);

	positional_data = std::vector<double>(sim_size * total_steps * 3);
	velocity_data = std::vector<double>(sim_size * total_steps * 3);
	acceleration_data = std::vector<double>(sim_size * total_steps * 3);

	std::random_device rnd_device;
	std::mt19937 mersenne_engine{ rnd_device() };
	std::uniform_int_distribution<int> dist{ -(box_size_angs*100)/2, (box_size_angs*100)/2 };
	auto gen_dist = [&dist, &mersenne_engine]()
	{
		return dist(mersenne_engine);
	};
	std::uniform_int_distribution<int> vel{ -5, 5 };
	auto gen_vel = [&vel, &mersenne_engine]()
	{
		return vel(mersenne_engine);
	};
	generate(begin(x_pos), end(x_pos), gen_dist);
	generate(begin(y_pos), end(y_pos), gen_dist);
	generate(begin(z_pos), end(z_pos), gen_dist);
	generate(begin(x_vel), end(x_vel), gen_vel);
	generate(begin(y_vel), end(y_vel), gen_vel);
	generate(begin(z_vel), end(z_vel), gen_vel);


	std::transform(x_pos.begin(), x_pos.end(), x_pos.begin(), [&particle_radius](double element) {return element *= particle_radius/100; });
	std::transform(y_pos.begin(), y_pos.end(), y_pos.begin(), [&particle_radius](double element) {return element *= particle_radius/100; });
	std::transform(z_pos.begin(), z_pos.end(), z_pos.begin(), [&particle_radius](double element) {return element *= particle_radius/100; });
	std::transform(x_vel.begin(), x_vel.end(), x_vel.begin(), [&particle_radius](double element) {return element *= particle_radius; });
	std::transform(y_vel.begin(), y_vel.end(), y_vel.begin(), [&particle_radius](double element) {return element *= particle_radius; });
	std::transform(z_vel.begin(), z_vel.end(), z_vel.begin(), [&particle_radius](double element) {return element *= particle_radius; });

	for (int i = 0; i < sim_size; i++)
	{
		if (i < sim_size/2)
		{
			charges[i] = 1;
		}
		else
		{
			charges[i] = -1;
		}
	}
}

std::vector<double> Fluid3D::get_x()
{
	return x_pos;
}

void Fluid3D::update_potentials(int step)
{
	//coulomb_acceleration();
}

void Fluid3D::coulomb_acceleration()
{
	for (int i = 0; i < sim_size; i++)
	{
		for (int j = 0; j < sim_size; j++)
		{
			if (i != j)
			{
				//(qq/4piE0)*1/r^2*m this gives incredibly large accelerations
				x_acc[i] = (magnetic_constant * charges[i] * charges[j]) / ((x_pos[i] - x_pos[j]) * (x_pos[i] - x_pos[j]));
				y_acc[i] = (magnetic_constant * charges[i] * charges[j]) / ((y_pos[i] - y_pos[j]) * (y_pos[i] - y_pos[j]));
				z_acc[i] = (magnetic_constant * charges[i] * charges[j]) / ((z_pos[i] - z_pos[j]) * (z_pos[i] - z_pos[j]));
			}
		}
	}
}

void Fluid3D::update_velocities(int step)
{
	for (int i = 0; i < sim_size; i++)
	{
		x_vel[i] = x_vel[i] + dt * x_acc[i];
		y_vel[i] = y_vel[i] + dt * y_acc[i];
		z_vel[i] = z_vel[i] + dt * z_acc[i];
	}
}

void Fluid3D::update_positions(int step)
{
	hits_box();
	double particle_radius = 1e-10;
	for (int i = 0; i < sim_size; i++)
	{
		for (int j = 0; j < sim_size; j++)
		{
			if (sqrt((x_pos[i]-x_pos[j])* (x_pos[i] - x_pos[j]) + (y_pos[i] - y_pos[j]) * (y_pos[i] - y_pos[j]) 
				+ (z_pos[i] - z_pos[j]) * (z_pos[i] - z_pos[j])) < 2*particle_radius && i != j)
			{
				//if two particles share a position then their velocities are updated to reflect a collision
				collisions += 1;
				x_vel[i] = (x_vel[i] + x_vel[j]) / 2;
				y_vel[i] = (y_vel[i] + y_vel[j]) / 2;
				z_vel[i] = (z_vel[i] + z_vel[j]) / 2;
				std::cout << "Collision between particle " << i << " and " << j << std::endl;
			}
		}
	}

	for (int i = 0; i < sim_size; i++)
	{
		//update particle positions
		x_pos[i] = x_pos[i] + x_vel[i] * dt;
		y_pos[i] = y_pos[i] + y_vel[i] * dt;
		z_pos[i] = z_pos[i] + z_vel[i] * dt;
		if (i == 0)
		{
			std::cout << "Particle 0 pos: " << x_pos[0] << " " << y_pos[0] << " " << z_pos[0] << std::endl;
			std::cout << "Particle 0 vel: " << x_vel[0] << " " << y_vel[0] << " " << z_vel[0] << std::endl;
		}
	}

	for (int i = 0; i < sim_size; i++)
	{
		//save data to output array
		positional_data[i * 3 + step * sim_size] = x_pos[i];
		positional_data[i * 3 + 1 + step * sim_size] = y_pos[i];
		positional_data[i * 3 + 2 + step * sim_size] = z_pos[i];
		velocity_data[i * 3 + step * sim_size] = x_vel[i];
		velocity_data[i * 3 + 1 + step * sim_size] = y_vel[i];
		velocity_data[i * 3 + 2 + step * sim_size] = z_vel[i];
		acceleration_data[i * 3 + step * sim_size] = x_acc[i];
		acceleration_data[i * 3 + 1 + step * sim_size] = y_acc[i];
		acceleration_data[i * 3 + 2 + step * sim_size] = z_acc[i];
	}
}

void Fluid3D::hits_box()
{
	double particle_radius = 1e-10;
	for (int i = 0; i < sim_size; i++)
	{
		if (abs(x_pos[i]) + particle_radius >= box_size / 2)
		{
			x_vel[i] = -x_vel[i];
			//cout << "Particle " << i << " collision with x wall "<< endl;
		}
		if (abs(y_pos[i]) + particle_radius >= box_size / 2)
		{
			y_vel[i] = -y_vel[i];
			//cout << "Particle " << i << " collision with y wall " << endl;
		}
		if (abs(z_pos[i]) + particle_radius >= box_size / 2)
		{
			z_vel[i] = -z_vel[i];
			//cout << "Particle " << i << " collision with z wall " << endl;
		}
	}
}

void Fluid3D::save_to_file(const int total_steps)
{
	std::ofstream f;
	std::filesystem::path filePath = current_path();
	filePath.append("fluid_data.csv");
	f.open(filePath.string());
	f << "sim_size," << "box_size," << "collisions," << "timesteps" << "\n";
	f << sim_size << "," << box_size_angs << "," << collisions << "," << total_steps << "\n";
	f << "Timestep";
	for (int j = 0; j < sim_size; j += 1)
	{
		f << ",x" << j << ",y" << j << ",z" << j << ",vx" << j << ",vy" << j << ",vz" << j << ",ax" << j << ",ay" << j << ",az" << j;
	}
	f << "\n";
	for (int i = 0; i < total_steps; i++)
	{
		f << i;
		for (int j = 0; j < sim_size * 3; j += 3)
		{
			f << "," << positional_data[j + sim_size * i] << "," << positional_data[j + sim_size * i + 1] << "," << positional_data[j + sim_size * i + 2];
			f << "," << velocity_data[j + sim_size * i] << "," << velocity_data[j + sim_size * i + 1] << "," << velocity_data[j + sim_size * i + 2];
			f << "," << acceleration_data[j + sim_size * i] << "," << acceleration_data[j + sim_size * i + 1] << "," << acceleration_data[j + sim_size * i + 2];
		}
		f << "\n";
	}
	f.close();
}

int main()
{
	//const int total_steps = 300;
	//Fluid3D p(total_steps);
	//int timestep = 0;
	//for (int step = 0; step <= total_steps; step++)
	//{
	//	std::vector<double> x_pos = p.get_x();
	//	p.update_potentials(step);
	//	p.update_velocities(step);
	//	p.update_positions(step);
	//	std::cout << "Time: " << step * p.dt << " seconds" << std::endl;
	//}
	//p.save_to_file(total_steps);
	//Laser l;
	//l.gaussian_intensity();
	Simulation sim;
	workhorse(sim);
	return 0;
}