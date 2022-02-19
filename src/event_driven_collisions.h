#pragma once
#include <vector>
#include <random>

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
private:
	New_Particle p1;
	New_Particle p2;

public:
	double collision_time;
	Event(New_Particle p1, New_Particle p2, double collision_time);
	void update_velocities();
	void update_particles(int i);
};

class Compute
{
private:
	std::vector<New_Particle> the_boys;
	double time;
	const double run_time = 1e-10;
	const double particle_radius = 1.5e-10;

public:
	double box_size = 50e-10;
	Random_setup random_setup;
	int particles = 1000;
	int max_events = 100;
	std::vector<Event> buddy_list;
	Compute();
	void add_particles();
	void cycle();
	void create_events();
	std::tuple<double, double> overlap(double, double, double, double);
	bool get_sign(double x);
	void order_event_list();
	void update_event_list();
	void update_particle_velocities();
};
