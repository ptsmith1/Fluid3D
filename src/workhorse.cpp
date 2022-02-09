#include <iostream>
#include <time.h>
#include <vector>
#include <algorithm>
#include <random>
#include <math.h>

#include "simulation.cpp"
#include "particle.h"

//extern int dt = 1;

void create_zones()
{

}

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

void random_vec_initialisation(std::vector<double>& vel)
{
	std::random_device rnd_device;
	std::mt19937 mersenne_engine{ rnd_device() };
	std::uniform_int_distribution<int> range{ -5, 5 };
	auto gen_vel = [&range, &mersenne_engine]()
	{
		return range(mersenne_engine);
	};
	generate(begin(vel), end(vel), gen_vel);
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
			double energy_absorbed =  dt * 30000 * (1e-10 * 1e-10 * 1e-10);
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

int workhorse(Simulation sim)
{
	double dt = 0.001;
	int runtime = 1; //seconds
	double currenttime = 0;
	double unit_size = 1e-10; //meters
	region.x = 100; //units
	region.y = 100;
	region.z = 100;
	int zone_size = 10; //units
	std::vector<double> pos = std::vector<double>(sim.d.region_x * region.y * region.z * 3); // stores x,y,z position of each particle
	std::vector<double> vel = std::vector<double>(region.x * region.y * region.z * 3);
	//std::vector<int> acc = std::vector<int>(region.x * region.y * region.z * 3);
	std::vector<int> particle_zones = std::vector<int>(region.x * region.y * region.z); //stores the zone that each particle is in
	std::vector<int> zones = std::vector<int>((region.x / zone_size) * (region.y / zone_size) * (region.z / zone_size)); //simple list of zones
	for (int z = 0; z < region.z; z++)
	{
		for (int y = 0; y < region.y; y++)
		{
			for (int x = 0; x < region.z; x++)
			{
				int i = (x + y * region.x + z * region.x * region.y);
				pos[i * 3] = x * unit_size;
				pos[i * 3 + 1] = y * unit_size;
				pos[i * 3 + 2] = z * unit_size;
			}
		}
	}
	for (int i = 0; i < zones.capacity(); i++)
	{
		zones[i] = i;
	}
	random_vec_initialisation(vel);


	while (currenttime < 1)
	{
		currenttime += dt;
		coords_to_zone_num(pos, zone_size, region, particle_zones); //updates each particles current zone
		update_pos(pos, vel, dt);
		heat_zone(pos, vel, particle_zones, dt);
	}
	return 0;
}