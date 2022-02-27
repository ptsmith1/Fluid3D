#include <iostream>
#include "event_driven_collisions.h"
#include <chrono>
#include <tuple>
#include <algorithm>
#include <fstream>
#include <filesystem>

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

Compute::Compute() : random_setup(300, -300, box_size, -box_size), popped_event(0,0,0,0)
{
	time = 0;
	add_particles();
	//Create each particles event and add to list in order of time
	for (New_Particle& p1 : the_boys)
	{
		create_event(p1);
	}
	order_event_list();
	save_data();
	cycle();
	save_events();
	save_data();
	data_to_csv();
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

void Compute::create_event(New_Particle& p1)
{
	//Create each particles event and add to list in order of time
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
			if (t1 >= 0 && t2 >= 0)
			{
				double t3 = 0, t4 = 0;
				double ydiff = p1.pos[1] - p2.pos[1];
				double vydiff = p1.vel[1] - p2.vel[1];
				t3 = (2 * particle_radius - ydiff) / vydiff;
				t4 = (-2 * particle_radius - ydiff) / vydiff;
				auto intermediate_interval = overlap(t1, t2, t3, t4);
				if (t3 >= 0 && t4 >= 0 && std::get<0>(intermediate_interval) != -1)
				{
					double t5 = 0, t6 = 0;
					double zdiff = p1.pos[2] - p2.pos[2];
					double vzdiff = p1.vel[2] - p2.vel[2];
					t5 = (2 * particle_radius - zdiff) / vzdiff;
					t6 = (-2 * particle_radius - zdiff) / vzdiff;
					auto valid_interval = overlap(std::get<0>(intermediate_interval), std::get<1>(intermediate_interval), t5, t6);
					if (std::get<0>(valid_interval) != -1)
					{
						double mid_time = (std::get<0>(intermediate_interval) + std::get<1>(intermediate_interval)) / 2; //valid if no acc
						double x_sep = (p1.pos[0] + mid_time * p1.vel[0]) - (p2.pos[0] + mid_time * p2.vel[0]);
						double y_sep = (p1.pos[1] + mid_time * p1.vel[1]) - (p2.pos[1] + mid_time * p2.vel[1]);
						double z_sep = (p1.pos[2] + mid_time * p1.vel[2]) - (p2.pos[2] + mid_time * p2.vel[2]);
						double sep_at_t = sqrt(x_sep * x_sep + y_sep * y_sep + z_sep * z_sep);
						if (sep_at_t <= 3e-10)
						{
							Event e(p1, p2, mid_time, mid_time);
							if (!check_dup_event(e)) { buddy_list.push_back(e); }
							std::cout << "Created event ID: " << e.id << std::endl;
							//double p1_x1 = p1.pos[0] + p1.vel[0] * std::get<0>(valid_interval);
							//double p1_y1 = p1.pos[1] + p1.vel[1] * std::get<0>(valid_interval);
							//double p1_z1 = p1.pos[2] + p1.vel[2] * std::get<0>(valid_interval);
							//double p2_x1 = p2.pos[0] + p2.vel[0] * std::get<0>(valid_interval);
							//double p2_y1 = p2.pos[1] + p2.vel[1] * std::get<0>(valid_interval);
							//double p2_z1 = p2.pos[2] + p2.vel[2] * std::get<0>(valid_interval);
							//double p1_x2 = p1.pos[0] + p1.vel[0] * std::get<1>(valid_interval);
							//double p1_y2 = p1.pos[1] + p1.vel[1] * std::get<1>(valid_interval);
							//double p1_z2 = p1.pos[2] + p1.vel[2] * std::get<1>(valid_interval);
							//double p2_x2 = p2.pos[0] + p2.vel[0] * std::get<1>(valid_interval);
							//double p2_y2 = p2.pos[1] + p2.vel[1] * std::get<1>(valid_interval);
							//double p2_z2 = p2.pos[2] + p2.vel[2] * std::get<1>(valid_interval);
							//std::cout << "Collision between " << std::get<0>(valid_interval) << " seconds and " << std::get<1>(valid_interval) << " seconds" << std::endl;
							//std::cout << "At t1, particle ID " << p1.id << " position: (" << p1_x1 << ", " << p1_y1 << ", " << p1_z1 << ") particle ID " << p2.id << " position : (" << p2_x1 << ", " << p2_y1 << ", " << p2_z1 << ")" << std::endl;
							//std::cout << "At t2, particle ID " << p1.id << " position: (" << p1_x2 << ", " << p1_y2 << ", " << p1_z2 << ") particle ID " << p2.id << " position : (" << p2_x2 << ", " << p2_y2 << ", " << p2_z2 << ")" << std::endl;
							//std::cout << "At t1, x,y,z seperation: (" << p1_x1 - p2_x1 << "," << p1_y1 - p2_y1 << "," << p1_z1 - p2_z1 << ")" << std::endl;
							//std::cout << "At t2, x,y,z seperation: (" << p1_x2 - p2_x2 << "," << p1_y2 - p2_y2 << "," << p1_z2 - p2_z2 << ")" << std::endl;
						}
					}
				}
			}
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
		return (e1.collision_time > e2.collision_time); //change > to order vector back to front
	}
} reorderer;

void Compute::order_event_list()
{
	std::sort(buddy_list.begin(), buddy_list.end(), reorderer);
}

void Compute::cycle()
{
	while (time < run_time && !buddy_list.empty())
	{
		//compute first event, invalidate some events, calculate new events for these particles, compute next event
		std::cout << "Before collision:" << std::endl;
		print_event_details(buddy_list.back());
		compute_event();
		update_event_list();
		order_event_list();
		update_event_times();
		std::cout << "After collision:" << std::endl;
		print_event_details(popped_event);
		time += popped_event.collision_time;
		std::cout << "This timestep = " << popped_event.collision_time << " Total time = " << time << std::endl;
		std::cout << std::endl;
		processed_events.push_back(popped_event);
	}
	end_time_update();
}

void Compute::compute_event()
{
	//this currently updates vel and pos of popped event particles as well as actual particles in the boys
	//dont currently need to update properties of popped event but may need to at some point
	Event e = buddy_list.back();
	e.time = e.collision_time;
	for (int i = 0; i < 3; i++)
	{
		e.p1.pos[i] += e.time_of_collision * e.p1.vel[i];
		e.p2.pos[i] += e.time_of_collision * e.p2.vel[i];
	}
	double x_sep = e.p1.pos[0] - e.p2.pos[0];
	double y_sep = e.p1.pos[1] - e.p2.pos[1];
	double z_sep = e.p1.pos[2] - e.p2.pos[2];
	double vx_sep = e.p1.vel[0] - e.p2.vel[0];
	double vy_sep = e.p1.vel[1] - e.p2.vel[1];
	double vz_sep = e.p1.vel[2] - e.p2.vel[2];

	e.p1.vel[0] += - x_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * particle_radius * particle_radius);
	e.p2.vel[0] += x_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * particle_radius * particle_radius);

	e.p1.vel[1] += - y_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * particle_radius * particle_radius);
	e.p2.vel[1] += y_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * particle_radius * particle_radius);

	e.p1.vel[2] += - z_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * particle_radius * particle_radius);
	e.p2.vel[2] += z_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * particle_radius * particle_radius);
	
	//updates particles in the boys
	the_boys[e.p1.id].vel = e.p1.vel;
	the_boys[e.p1.id].pos = e.p1.pos;
	the_boys[e.p2.id].vel = e.p2.vel;
	the_boys[e.p2.id].pos = e.p2.pos;

	popped_event = e;
	e.p1.last_collision_time = time + e.collision_time; //time of this collision
	e.p2.last_collision_time = time + e.collision_time;

	buddy_list.pop_back();
	collisions++;
	std::cout << "Popped event: " << popped_event.id << std::endl;
}


void Compute::update_event_list()
{
	create_event(popped_event.p1);
	create_event(popped_event.p2);
	for (Event e : buddy_list)
	{
		if (popped_event.p1.id == e.p1.id || popped_event.p2.id == e.p1.id)
		{
			//new events for particles that would have hit the collided particles had they not collided
			create_event(e.p1);
			create_event(e.p2);
		}
	}
}

bool Compute::check_dup_event(Event new_event)
{ //return true if id of new event matches the id/reverse id of event in event list
	for (Event &e : buddy_list)
	{
		std::string reversed(new_event.id.rbegin(), new_event.id.rend());
		if (e.id == new_event.id || e.id == reversed) {return true;}
	}
	return false;
}

void Compute::update_event_times()
{
	for (Event& e : buddy_list)
	{
		e.collision_time = e.collision_time - popped_event.collision_time;
	}
}

void Compute::print_event_details(Event e)
{
	std::cout << "ID: " << e.id << std::endl;
	std::cout << "Particle " << e.p1.id << " position: (" << e.p1.pos[0] << ", " << e.p1.pos[1] << ", " << e.p1.pos[2] << ") velocity: (" << e.p1.vel[0] << ", " << e.p1.vel[1] << ", " << e.p1.vel[2] << ")" << std::endl;
	std::cout << "Particle " << e.p2.id << " position : (" << e.p2.pos[0] << ", " << e.p2.pos[1] << ", " << e.p2.pos[2] << ") velocity : (" << e.p2.vel[0] << ", " << e.p2.vel[1] << ", " << e.p2.vel[2] << ")" << std::endl;
}

void Compute::end_time_update()
{
	for (New_Particle& p : the_boys)
	{
		double travel_time = time - p.last_collision_time;
		for (int i = 0; i < 3; i++)
		{
			p.pos[i] += p.vel[i] * travel_time;
		}
	}
}

void Compute::save_events()
{
	std::reverse(processed_events.begin(), processed_events.end());
	double save_time = 0;
	int it = 0;
	while (save_time < run_time)
	{
		it++;
		save_time += time_save_interval;
		int offset = (particles * 6) * (it-1);
		for (New_Particle p : the_boys)
		{
			particle_data.push_back(particle_data[offset + p.id * 6] + time_save_interval * particle_data[offset + (p.id * 6)+3]);
			particle_data.push_back(particle_data[offset + (p.id * 6)+1] + time_save_interval * particle_data[offset + (p.id * 6)+4]);
			particle_data.push_back(particle_data[offset + (p.id * 6)+2] + time_save_interval * particle_data[offset + (p.id * 6)+5]);
			particle_data.push_back(particle_data[offset + (p.id * 6)+3]);
			particle_data.push_back(particle_data[offset + (p.id * 6)+4]);
			particle_data.push_back(particle_data[offset + (p.id * 6)+5]);
		}
		offset = (particles * 6) * (it);
		for (Event e : processed_events)
		{
			if (e.time < save_time && (save_time - e.time) < time_save_interval)
			{
				//if an event time has been reached, then the particles in the event have their saved pos/vel updated in the next save 
				//timestep. This overwrites the previous data which assumed no collision had occurred. 
				std::list<New_Particle> event_particles = { e.p1,e.p2 };
				for (New_Particle p : event_particles)
				{
					particle_data[offset +(p.id * 6)] = p.pos[0] + p.vel[0] * (save_time - e.time);
					particle_data[offset +(p.id * 6) + 1] = p.pos[1] + p.vel[1] * (save_time - e.time);
					particle_data[offset +(p.id * 6) + 2] = p.pos[2] + p.vel[2] * (save_time - e.time);
					particle_data[offset +(p.id * 6) + 3] = p.vel[0];
					particle_data[offset +(p.id * 6) + 4] = p.vel[1];
					particle_data[offset +(p.id * 6) + 5] = p.vel[2];
				}
			}
		}
	}
}

void Compute::save_data()
{
	//saves current particle positions and velocities to array
	for (New_Particle p : the_boys)
	{
		particle_data.push_back(p.pos[0]);
		particle_data.push_back(p.pos[1]);
		particle_data.push_back(p.pos[2]);
		particle_data.push_back(p.vel[0]);
		particle_data.push_back(p.vel[1]);
		particle_data.push_back(p.vel[2]);
	}
}

void Compute::data_to_csv()
{
	std::ofstream f;
	std::filesystem::path filePath = std::filesystem::current_path();
	filePath.append("fluid_data.csv");
	f.open(filePath.string());
	f << "sim_size," << "box_size," << "collisions," << "timesteps" << "\n";
	f << particles << "," << box_size << "," << collisions << "," << (run_time / time_save_interval) + 1 << "\n";
	f << "Timestep";
	for (int j = 0; j < particles; j += 1)
	{
		f << ",x" << j << ",y" << j << ",z" << j << ",vx" << j << ",vy" << j << ",vz" << j;
	}
	f << "\n";

	for (int i = 0; i < (run_time / time_save_interval) + 1; i++)
	{
		f << i;
		for (int j = 0; j < (particles * 6); j ++)
		{
			f << "," << particle_data[j + particles * 6 * i];
		
		}
		f << "\n";
	}
	f.close();
}


Event::Event(New_Particle p1, New_Particle p2, double collision_time, double time_of_collision) :p1(p1), p2(p2), collision_time(collision_time), time_of_collision(time_of_collision)
{
	id = std::to_string(p1.id) + "-" + std::to_string(p2.id);
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
	std::cout << "OOP method time:" << (std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count()) * 1e-9 << std::endl;
	//asdf
	//sdaf
	//fa
	//dsff
	//af
	return 0;
}
