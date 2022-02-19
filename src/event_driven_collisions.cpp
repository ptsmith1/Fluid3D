#include <iostream>
#include "event_driven_collisions.h"
#include <chrono>
#include <tuple>
#include <algorithm>

typedef std::chrono::high_resolution_clock Clock;

Random_setup::Random_setup(double maxv, double minv, double maxp, double minp)
{
	pos_range = std::uniform_real_distribution<double>{ minp, maxp };
	vel_range = std::uniform_real_distribution<double>{ minv, maxv };
}

New_Particle::New_Particle(int id)
{
	this->id = id;
	pos = std::vector<double>(3);
	vel = std::vector<double>(3);
}

void New_Particle::init_vectors(Random_setup& random_setup)
{
	auto pos_range = random_setup.pos_range;
	auto vel_range = random_setup.vel_range;
	std::random_device rnd_device;
	std::mt19937 mersenne_engine{ rnd_device() };
	auto gen_pos = [&pos_range, &mersenne_engine]()
	{
		return pos_range(mersenne_engine);
	};
	auto gen_vel = [&vel_range, &mersenne_engine]()
	{
		return vel_range(mersenne_engine);
	};
	generate(begin(pos), end(pos), gen_pos);
	generate(begin(vel), end(vel), gen_vel);
}

Compute::Compute() : random_setup(300, -300, box_size, -box_size)
{
	time = 0;
	add_particles();
	create_events();
	order_event_list();
}

void Compute::add_particles()
{
	for (int i = 0; i < particles; i++)
	{
		New_Particle p(i);
		p.init_vectors(random_setup);
		the_boys.push_back(p);
	}
}

void Compute::create_events()
{
	//Create each particles event and add to list in order of time
	for (New_Particle& p1 : the_boys)
	{
		for (New_Particle& p2 : the_boys)
		{
			if (p2.id < p1.id)
			{
				double time_to_collision = 0;
				double t1 = 0, t2 = 0;
				double xdiff = p1.pos[0] - p2.pos[0];
				double vxdiff = p1.vel[0] - p2.vel[0];
				t1 = (2 * particle_radius - xdiff) / vxdiff;
				t2 = (-2 * particle_radius - xdiff) / vxdiff;	
				double x1_at_t1 = (p1.pos[0] + t1 * p1.vel[0]); //temp
				double x2_at_t1 = (p2.pos[0] + t1 * p2.vel[0]); //temp
				double xsep_at_t1 = x2_at_t1 - x1_at_t1; //temp
				if (t1 >=0 && t2 >=0 )
				{
					double t3 = 0, t4 = 0;
					double ydiff = p1.pos[1] - p2.pos[1];
					double vydiff = p1.vel[1] - p2.vel[1];
					t3 = (2 * particle_radius - ydiff) / vydiff;
					t4 = (-2 * particle_radius - ydiff) / vydiff;
					auto intermediate_interval = overlap(t1, t2, t3, t4);
					double y1_at_t3 = (p1.pos[1] + t3 * p1.vel[1]); //temp
					double y2_at_t3 = (p2.pos[1] + t3 * p2.vel[1]); //temp
					double ysep_at_t3 = y2_at_t3 - y1_at_t3; //temp
					if (t3 >= 0 && t4 >= 0 && std::get<0>(intermediate_interval)!=-1)
					{
						double t5 = 0, t6 = 0;
						double zdiff = p1.pos[2] - p2.pos[2];
						double vzdiff = p1.vel[2] - p2.vel[2];
						t5 = (2 * particle_radius - zdiff) / vzdiff;
						t6 = (-2 * particle_radius - zdiff) / vzdiff;
						auto valid_interval = overlap(std::get<0>(intermediate_interval), std::get<1>(intermediate_interval), t5, t6);
						double z1_at_t5 = (p1.pos[2] + t5 * p1.vel[2]); //temp
						double z2_at_t5 = (p2.pos[2] + t5 * p2.vel[2]); //temp
						double zsep_at_t5 = z2_at_t5 - z1_at_t5; //temp
						if (std::get<0>(valid_interval) != -1)
						{
							Event e(p1, p2, std::get<0>(valid_interval));
							buddy_list.push_back(e);
							double p1_x1 = p1.pos[0] + p1.vel[0] * std::get<0>(valid_interval);
							double p1_y1 = p1.pos[1] + p1.vel[1] * std::get<0>(valid_interval);
							double p1_z1 = p1.pos[2] + p1.vel[2] * std::get<0>(valid_interval);
							double p2_x1 = p2.pos[0] + p2.vel[0] * std::get<0>(valid_interval);
							double p2_y1 = p2.pos[1] + p2.vel[1] * std::get<0>(valid_interval);
							double p2_z1 = p2.pos[2] + p2.vel[2] * std::get<0>(valid_interval);
							double p1_x2 = p1.pos[0] + p1.vel[0] * std::get<1>(valid_interval);
							double p1_y2 = p1.pos[1] + p1.vel[1] * std::get<1>(valid_interval);
							double p1_z2 = p1.pos[2] + p1.vel[2] * std::get<1>(valid_interval);
							double p2_x2 = p2.pos[0] + p2.vel[0] * std::get<1>(valid_interval);
							double p2_y2 = p2.pos[1] + p2.vel[1] * std::get<1>(valid_interval);
							double p2_z2 = p2.pos[2] + p2.vel[2] * std::get<1>(valid_interval);
							std::cout << "Collision between " << std::get<0>(valid_interval) << " seconds and " << std::get<1>(valid_interval) << " seconds" << std::endl;
							std::cout << "At t1, particle ID "<<p1.id<<" position: (" << p1_x1 <<", "<< p1_y1<<", "<< p1_z1 << ") particle ID " << p2.id << " position : (" << p2_x1 << ", " << p2_y1 << ", " << p2_z1 << ")"<<std::endl;
							std::cout << "At t2, particle ID " << p1.id << " position: (" << p1_x2 << ", " << p1_y2 << ", " << p1_z2 << ") particle ID " << p2.id << " position : (" << p2_x2 << ", " << p2_y2 << ", " << p2_z2 << ")" << std::endl;
							std::cout << "At t1, x,y,z seperation: (" << p1_x1 - p2_x1 << "," << p1_y1 - p2_y1 << "," << p1_z1 - p2_z1 << ")" << std::endl;
							std::cout << "At t2, x,y,z seperation: (" << p1_x2 - p2_x2 << "," << p1_y2 - p2_y2 << "," << p1_z2 - p2_z2 << ")" << std::endl;
						}
					}
				}
			}
			//std::cout << p1.id << "," << p2.id << std::endl;
		}
	}
}

std::tuple<double, double> Compute::overlap(double t1, double t2, double t3, double t4)
{
	//takes to time periods t1-t2 and t3-t4 and returns any overlap time period
	if (t1 > t2)
	{
		double temp = t2;
		t2 = t1;
		t1 = temp;
	}
	if (t3 > t4)
	{
		double temp = t4;
		t4 = t3;
		t3 = temp;
	}
	if (t3 < t2 && t3 > t1 && t4 > t2)
	{
		return std::make_tuple(t3, t2);
	}
	else if (t1 < t4 && t1 > t3 && t2 > t4)
	{
		return std::make_tuple(t1, t4);
	}
	else if (t1 < t3 && t2 > t4)
	{
		return std::make_tuple(t3, t4);
	}
	else if (t1 > t3 && t4 > t2)
	{
		return std::make_tuple(t1, t2);
	}
	else
	{
		return std::make_tuple(-1, -1);
	}
}

bool Compute::get_sign(double x)
{
	if (x >= 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

struct reorder
{
	bool operator() (Event e1, Event e2) 
	{ 
		return (e1.collision_time < e2.collision_time); //i dont realy understand how this works
	}
} reorderer;

void Compute::order_event_list()
{
	std::sort(buddy_list.begin(), buddy_list.end(), reorderer);
}

void Compute::update_event_list()
{
	
}

void Compute::cycle()
{
	while (time < run_time)
	{
		time = 100;
		//compute first event, invalidate some events, calculate new events for these particles, compute next event
		update_event_list();
	}
}


Event::Event(New_Particle p1, New_Particle p2, double collision_time) :p1(p1), p2(p2), collision_time(collision_time)
{

}

void Event::update_particles(int i)
{
	p1 = i;
	p2 = i;
}

int main_func()
{
	auto t1 = Clock::now();
	Compute compute;
	compute.cycle();
	auto t2 = Clock::now();
	//Event event(-1,-1);
	//for (int i = 0; i < compute.max_events; i++)
	//{
	//	event.update_paraticles(i);
	//	compute.buddy_list.push_back(event);
	//}
	std::cout << "OOP method time:" << (std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count()) * 1e-9 << std::endl;
	return 0;
}
