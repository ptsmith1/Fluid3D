#pragma once
#include "random_setup.h"
#include <vector>

struct Box
{
	double _x_boundary = 20e-10;
	double _y_boundary = 20e-10;
	double _z_boundary = 20e-10;
};

class Particle
{
public:
	int _id;
	double _last_collision_time = 0;
	std::vector<double> _pos;
	std::vector<double> _vel;
	Particle(int _id);
	Particle();
	void init_vectors(Random_Setup & _random_setup);
};

class Event
{
	//records the time of a collision event and the position of particles when the event occurs
	//on creation they hold the starting pos/vel of particle, on execution of event the pos/vel 
	//are updated to the values immediately after collision
public:
	std::string _id;
	std::string _type;
	int _axis_hit = 0;
	int _null = 1;
	Particle _p1;
	Particle _p2;
	double _sim_time = 0; //time into the simulation
	double _current_time_to_collision = 0; //stores how long the simulation timer must be advanced to reach collision
	double _original_time_to_collision = 0; //stores how long the two particles have travelled for before colliding
	Event();
	Event(int null);
	Event(Particle p1, Particle p2, double collision_time, double time_of_collision);
	Event(Particle p, int axis_hit, double collision_time, double time_of_collision);
};


class Simulation {
public:
	int _particles = 100;
	int _collisions = 0;
	double _sim_time = 0;
	const double _run_time = 1e-11;
	const double _particle_radius = 1.5e-10;
	double _dt = 1e-14;
	double _time_save_interval = 1e-13;
	int steps = _run_time / _dt;
	std::string _type;
	Box _box;
	double _box_size = _box._x_boundary;
	Event *_popped_event;
	Random_Setup *_random_setup = new Random_Setup(300, -300, _box._x_boundary, -_box._x_boundary);
	std::vector<Particle> _particle_array;
	std::vector<double> _particle_data;
	std::vector<Event> _event_list;
	std::vector<Event> _processed_events;
	Simulation();
	Simulation(std::string);
	void add_random_particles();
	void order_event_list();
	void add_test_particles();
};
