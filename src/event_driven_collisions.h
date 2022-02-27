#pragma once
#include <vector>
#include <random>
#include <string>

class Random_setup
{
public:
	std::uniform_real_distribution<double> pos_range;
	std::uniform_real_distribution<double> vel_range;
	Random_setup(double maxv, double minv, double maxp, double minp);
};

class New_Particle
{
private:
public:
	int id;
	double last_collision_time = 0;
	std::vector<double> pos;
	std::vector<double> vel;
	New_Particle(int id);
	void init_vectors(Random_setup &random_setup);
	std::vector<double> get_pos();
	std::vector<double> get_vel();
	void set_pos(std::vector<double>& pos);
	void set_vel(std::vector<double>& vel);
};

class Event
{
public:
	std::string id;
	New_Particle p1;
	New_Particle p2;
	double time; //time into the simulation
	double collision_time; //stores how long the simulation timer must be advanced to reach collision
	double time_of_collision; //stores how long the two particles have travelled for before colliding
	Event(New_Particle p1, New_Particle p2, double collision_time, double time_of_collision);
	void update_velocities();
	void update_particles(int i);
};

class Compute
{
private:
	std::vector<New_Particle> the_boys;
	double time;
	const double run_time = 1e-11;
	const double particle_radius = 1.5e-10;
	Event popped_event;

public:
	double box_size = 20e-10;
	double time_save_interval = 1e-14;
	Random_setup random_setup;
	int particles = 100;
	int collisions = 0;
	std::vector<double> particle_data;
	std::vector<Event> buddy_list;
	std::vector<Event> processed_events;
	Compute();
	void add_particles();
	void cycle();
	void save_data();
	void save_events();
	void create_event(New_Particle& p1);
	std::tuple<double, double> overlap(double, double, double, double);
	bool get_sign(double x);
	void order_event_list();
	void compute_event();
	void update_event_list();
	void update_particle_velocities();
	bool check_dup_event(Event new_event);
	void update_event_times();
	void print_event_details(Event e);
	void end_time_update();
	void data_to_csv();
};

int main_func();