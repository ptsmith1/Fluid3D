// Fluid3D.cpp : Defines the entry point for the application.
//

#include "Fluid3D.h"
#include <memory>
#include <vector>
#include <random>
#include <iostream>
#include <fstream>
#include <filesystem>

using namespace std;

class particles
{
private:
	vector<int> x_pos;
	vector<int> y_pos;
	vector<int> z_pos;

	vector<int> x_vel;
	vector<int> y_vel;
	vector<int> z_vel;
	
	const int box_size = 100;
	const int sim_size = 100;
	int collisions = 0;

	vector<int> positional_data;
	vector<int> velocity_data;



public:
	const int dt = 1; //1 second
	particles(const int total_steps)
	{
		x_pos = vector<int>(sim_size);
		y_pos = vector<int>(sim_size);
		z_pos = vector<int>(sim_size);
		x_vel = vector<int>(sim_size);
		y_vel = vector<int>(sim_size);
		z_vel = vector<int>(sim_size);

		positional_data = vector<int>(sim_size * total_steps * 3);
		velocity_data = vector<int>(sim_size * total_steps * 3);

		random_device rnd_device;
		mt19937 mersenne_engine{ rnd_device() };
		uniform_int_distribution<int> dist{ -49, 49 };
		auto gen_dist = [&dist, &mersenne_engine]()
		{
			return dist(mersenne_engine);
		};
		uniform_int_distribution<int> vel{ -5, 5 };
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
	}

	vector<int> get_x()
	{
		return x_pos;
	}

	vector<int> get_y()
	{
		return y_pos;
	}

	vector<int> get_z()
	{
		return z_pos;
	}

	void update_positions(int step)
	{
		hits_box();
		for (int i = 0; i < sim_size; i++)
		{
			for (int j = 0; j < sim_size; j++)
			{
				if (x_pos[i] == x_pos[j] && y_pos[i] == y_pos[j] && z_pos[i] == z_pos[j] && i!=j)
				{
					//if two particles share a position then their velocities are updated to reflect a collision
					collisions += 1;
					x_vel[i] = (x_vel[i] + x_vel[j]) / 2;
					y_vel[i] = (y_vel[i] + y_vel[j]) / 2;
					z_vel[i] = (z_vel[i] + z_vel[j]) / 2;
					cout << "Collision between particle " << i << " and " << j << endl;
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
				cout << "Particle 0 pos: " << x_pos[0] << " " << y_pos[0] << " " << z_pos[0] << endl;
				cout << "Particle 0 vel: " << x_vel[0] << " " << y_vel[0] << " " << z_vel[0] << endl;
			}
		}

		for (int i = 0; i < sim_size; i++)
		{
			//save data to output array
			positional_data[i*3 + step*sim_size] = x_pos[i];
			positional_data[i*3 + 1 + step * sim_size] = y_pos[i];
			positional_data[i*3 + 2 + step * sim_size] = z_pos[i];
			velocity_data[i * 3 + step * sim_size] = x_vel[i];
			velocity_data[i * 3 + 1 + step * sim_size] = y_vel[i];
			velocity_data[i * 3 + 2 + step * sim_size] = z_vel[i];
		}
	}

	void hits_box()
	{
		for (int i = 0; i < sim_size; i++)
		{
			if (abs(x_pos[i]) >= box_size / 2)
			{
				x_vel[i] = -x_vel[i];
				//cout << "Particle " << i << " collision with x wall "<< endl;
			}
			if (abs(y_pos[i]) >= box_size / 2)
			{
				y_vel[i] = -y_vel[i];
				//cout << "Particle " << i << " collision with y wall " << endl;
			}
			if (abs(z_pos[i]) >= box_size / 2)
			{
				z_vel[i] = -z_vel[i];
				//cout << "Particle " << i << " collision with z wall " << endl;
			}
		}
	}

	void save_to_file(const int total_steps)
	{
		ofstream f;
		f.open("C:\\Users\\Philip\\Desktop\\fluid_data.csv");
		f << "sim_size," << "box_size," << "collisions," << "timesteps" << "\n";
		f << sim_size << "," << box_size << "," << collisions << "," << total_steps << "\n";
		f << "Timestep";
		for (int j = 0; j < sim_size; j += 1)
		{
			f << ",x" << j << ",y" << j << ",z" << j << ",vx" << j << ",vy" << j << ",vz" << j;
		}
		f << "\n";
		for (int i = 0; i < total_steps; i ++)
		{
			f << i;
			for (int j = 0; j < sim_size * 3; j +=3)
			{
				f << "," << positional_data[j + sim_size * i] << "," << positional_data[j + sim_size * i + 1] << "," << positional_data[j + sim_size * i + 2];
				f << "," << velocity_data[j + sim_size * i] << "," << velocity_data[j + sim_size * i + 1] << "," << velocity_data[j + sim_size * i + 2];
			}
			f << "\n";
		}
		f.close();
	}

};

void functionUsingWidget() 
{
	const int total_steps = 100;
    particles p(total_steps);
	int timestep = 0;
	for (int step = 0; step<=total_steps; step++)
	{
		vector<int> x_pos = p.get_x();
		p.update_positions(step);
		cout << "Time: " << step * p.dt << " seconds" << endl;
	}
	p.save_to_file(total_steps);
} 


int main()
{
	functionUsingWidget();
	return 0;
}
