#include <iostream>
#include <time.h>
#include <vector>
#include <algorithm>
#include <random>
#include <math.h>

#include "simulation.h"
#include "particle.h"


struct region_dims
{
	int x;
	int y;
	int z;
}region;

void coords_to_zone_num(std::vector<double> pos, int zone_size, region_dims region, std::vector<int>& particle_zones)
{
	int x_regions = region.x / 10;
	int xy_regions = x_regions*(region.y / 10);
	for (int z = 0; z < 100; z++)
	{
		for (int y = 0; y < 100; y++)
		{
			for (int x = 0; x < 100; x++)
			{
				int i = (x + y * region.x + z * region.x * region.y);
				int x_zone = pos[i * 3] / zone_size;
				int y_zone = pos[i * 3 + 1] / zone_size;
				int z_zone = pos[i * 3 + 2] / zone_size;
				int zone = x_zone + x_regions * y_zone + xy_regions * z_zone;
				particle_zones[i] = zone;
				//std::cout << pos[(x + y * region.x + z * region.x * region.y) * 3] << ","<< pos[(x + y * region.x + z * region.x * region.y) * 3 + 1] << "," << pos[(x + y * region.x + z * region.x * region.y) * 3 + 2] << "," << zone << std::endl;
			}
		}
	}
}

void initiate_vel_vec(std::vector<double>& vec, int max, int min)
{
	std::random_device rnd_device;
	std::mt19937 mersenne_engine{ rnd_device() };
	std::uniform_int_distribution<int> range{ min, max };
	auto gen_vel = [&range, &mersenne_engine]()
	{
		return range(mersenne_engine);
	};
	generate(begin(vec), end(vec), gen_vel);
	std::cout << std::endl;
}

auto random_number(double min, double max)
{
	std::random_device rnd_device;
	std::mt19937 mersenne_engine{ rnd_device() };
	std::uniform_real_distribution<double> range{ min, max };
	return range(mersenne_engine);
}

void initiate_pos_vec(std::vector<double>& vec, double x, double y, double z)
{
	for (int i = 0; i < vec.capacity(); i+=3)
	{
		vec[i] = random_number(-x, x);
		vec[i+1] = random_number(-y, y);
		vec[i+2] = random_number(-z, z);
	}
}

void update_pos(std::vector<double>& pos, std::vector<double>& vel, double dt)
{
	for (int i = 0; i<pos.size();i++)
	{
		pos[i] = pos[i] + vel[i] * dt;
	}
}

void heat_zone(std::vector<double>& pos, std::vector<double>& vel, std::vector<int>& particle_zones, double dt)
{
	for (int i = 0; i < pos.size(); i++)
	{
		if (particle_zones[i / 3] == 0)
		{
			double energy_absorbed =  dt * 30000 * (3e-10 * 3e-10 * 3e-10);
			if (vel[i] >= 0)
			{
				vel[i] += sqrt((2 * energy_absorbed) / 1.67e-27);
			}
			else
			{
				vel[i] += -sqrt((2 * energy_absorbed) / 1.67e-27);
			}
		}
	}
}

void wall_collisions(Simulation& sim, std::vector<double>& pos, std::vector<double>& vel)
{
	for (int i = 0; i < pos.capacity(); i += 3)
	{
		if (pos[i] < -sim.d.material_type.width / 2 || pos[i] > sim.d.material_type.width / 2)
		{
			vel[i] = -vel[i];
		}
		if (pos[i+1] < -sim.d.material_type.height / 2 || pos[i+1] > sim.d.material_type.height / 2)
		{
			vel[i+1] = -vel[i+1];
		}
		if (pos[i+2] < -sim.d.material_type.depth / 2 || pos[i+2] > sim.d.material_type.depth / 2)
		{
			vel[i+2] = -vel[i+2];
		}
	}
	std::cout << "Wall collision" << std::endl;
}

int particle_collisions(Simulation& sim, std::vector<double>& pos, std::vector<double>& vel, int collision_count)
{
	for (int n = 0; n < pos.capacity(); n += 3)
	{
		for (int m = 0; m < pos.capacity(); m += 3)
		{
			if (m > n)
			{
				double x_sep = pos[n] - pos[m];
				double y_sep = pos[n + 1] - pos[m + 1];
				double z_sep = pos[n + 2] - pos[m + 2];
				double seperation = sqrt(x_sep * x_sep + y_sep * y_sep + z_sep * z_sep);
				if (abs(seperation) < 2 * sim.particle_radius)
				{
					double vx_sep = vel[n] - vel[m];
					double vy_sep = vel[n + 1] - vel[m + 1];
					double vz_sep = vel[n] - vel[m];

					vel[n] = vel[n] - x_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * sim.particle_radius * sim.particle_radius);
					vel[m] = vel[m] + x_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * sim.particle_radius * sim.particle_radius);

					vel[n + 1] = vel[n + 1] - y_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * sim.particle_radius * sim.particle_radius);
					vel[m + 1] = vel[m + 1] + y_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * sim.particle_radius * sim.particle_radius);

					vel[n + 2] = vel[n + 2] - z_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * sim.particle_radius * sim.particle_radius);
					vel[m + 2] = vel[m + 2] + z_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * sim.particle_radius * sim.particle_radius);

					std::cout << "Particle Collision" << std::endl;
					collision_count++;

				}
			}
		}
	}
	return collision_count;
	
}


std::tuple<std::vector<double>, std::vector<double>, int> workhorse(Simulation sim)
{
	int collision_count = 0;
	double currenttime = 0;
	double unit_size = 1e-10; //meters
	double zone_count = sim.d.region_x * sim.d.region_y * sim.d.region_z;

	std::vector<double> pos = std::vector<double>(sim.particle_count * 3); // stores x,y,z position of each particle
	std::vector<double> vel = std::vector<double>(sim.particle_count * 3);

	std::vector<double> all_pos;
	std::vector<double> all_vel;
	//std::vector<int> particle_zones = std::vector<int>(sim.particle_count); //stores the zone that each particle is in
	//std::vector<int> zones = std::vector<int>(zone_count); //simple list of zones

	//for (int z = 0; z < sim.d.region_z; z++)
	//{
	//	for (int y = 0; y < sim.d.region_y; y++)
	//	{
	//		for (int x = 0; x < sim.d.region_x; x++)
	//		{
	//			int i = (x + y * sim.d.region_x + z * sim.d.region_x * sim.d.region_y);
	//			pos[i * 3] = x * unit_size;
	//			pos[i * 3 + 1] = y * unit_size;
	//			pos[i * 3 + 2] = z * unit_size;
	//		}
	//	}
	//}
	//for (int i = 0; i < zones.capacity(); i++)
	//{
	//	zones[i] = i;
	//}

	initiate_vel_vec(vel, 500,-500);
	initiate_pos_vec(pos, sim.d.material_type.width/2, sim.d.material_type.height/2, sim.d.material_type.depth/2); //should realy pass position boundaries based on material size and origin

	while (currenttime < sim.run_time)
	{
		currenttime += sim.dt;
		all_pos.insert(all_pos.end(), pos.begin(), pos.end());
		all_vel.insert(all_vel.end(), vel.begin(), vel.end());
		//coords_to_zone_num(pos, sim.d.zone_size, region, particle_zones); //updates each particles current zone
		update_pos(pos, vel, sim.dt);
		wall_collisions(sim, pos, vel);
		collision_count = particle_collisions(sim, pos, vel, collision_count);
		//heat_zone(pos, vel, particle_zones, dt);
	}

	return std::make_tuple(all_pos, all_vel, collision_count);
}