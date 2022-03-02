#pragma once
#include <vector>
#include <random>
#include <string>

class Random_setup
{
public:
	std::uniform_real_distribution<double> _pos_range;
	std::uniform_real_distribution<double> _vel_range;
	Random_setup(double maxv, double minv, double maxp, double minp);
};

class New_Particle
{
private:
public:
	int _id;
	double _last_collision_time = 0;
	std::vector<double> _pos;
	std::vector<double> _vel;
	New_Particle(int _id);
	void init_vectors(Random_setup &_random_setup);
};

struct Box
{
	double _x_boundary = 20e-10;
	double _y_boundary = 20e-10;
	double _z_boundary = 20e-10;
};

class Event
{
public:
	std::string _id;
	std::string _type;
	int _axis_hit;
	New_Particle _p1;
	New_Particle _p2;
	Box _box;
	double _time; //time into the simulation
	double _collision_time; //stores how long the simulation timer must be advanced to reach collision
	double _time_of_collision; //stores how long the two particles have travelled for before colliding
	Event(New_Particle p1, New_Particle p2, double collision_time, double time_of_collision);
	Event(New_Particle p, int axis_hit, double collision_time, double time_of_collision);
	void update_particles(int i);
};

class Compute
{
private:
	std::vector<New_Particle> _the_boys;
	double _time;
	const double _run_time = 1e-11;
	const double _particle_radius = 1.5e-10;
	Event _popped_event;

public:
	Box _box;
	double _box_size = _box._x_boundary;
	double _time_save_interval = 1e-13;
	Random_setup _random_setup;
	int _particles = 100;
	int _collisions = 0;
	std::vector<double> _particle_data;
	std::vector<Event> _buddy_list;
	std::vector<Event> _processed_events;
	Compute();
	void add_particles();
	void cycle();
	void save_data();
	void save_events();
	void create_collision_event(New_Particle _p1);
	void create_wall_event(New_Particle _p);
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

int test_function();

int main_func();