#include "simulation.h"
#include <string>
#include <vector>
#include <iostream>

Particle::Particle()
{

}

Particle::Particle(int id)
{
	_id = id;
    _pos = std::vector<double>(3);
	_vel = std::vector<double>(3);
}

void Particle::init_vectors(Random_Setup& _random_setup)
{
	auto _pos_range = _random_setup._pos_range;
	auto _vel_range = _random_setup._vel_range;
	std::random_device rnd_device;
	std::mt19937 mersenne_engine{ rnd_device() };
	auto gen_pos = [&_pos_range, &mersenne_engine]()
	{
		return _pos_range(mersenne_engine);
	};
	auto gen_vel = [&_vel_range, &mersenne_engine]()
	{
		return _vel_range(mersenne_engine);
	};
	generate(begin(_pos), end(_pos), gen_pos);
	generate(begin(_vel), end(_vel), gen_vel);
}

struct reorder
{
	bool operator() (Event e1, Event e2)
	{
		return (e1._current_time_to_collision > e2._current_time_to_collision); //change > to order vector back to front
	}
} reorderer;

Event::Event()
{
	_p1 = NULL;
	_p2 = NULL;
}

Event::Event(Particle p1, Particle p2, double collision_time, double time_of_collision) :_p1(p1), _p2(p2), _current_time_to_collision(collision_time), _original_time_to_collision(time_of_collision)
{
	//collision event
	_type = "particle";
	_id = std::to_string(_p1._id) + "-" + std::to_string(_p2._id);
}

Event::Event(Particle p, int axis_hit, double collision_time, double time_of_collision) : _p1(p), _axis_hit(axis_hit), _p2(NULL), _current_time_to_collision(collision_time), _original_time_to_collision(time_of_collision)
{
	//wall event
	_type = "wall";
	_id = std::to_string(_p1._id) + "-wall";
}

Event::Event(int null)
{
	_null = 0;
}

Simulation::Simulation() : _random_setup(300, -300, _box_size, -_box_size)
{
	_sim_time = 0;
	add_particles();
}

void Simulation::add_particles()
{
	for (int i = 0; i < _particles; i++)
	{
		Particle p(i);
		p.init_vectors(_random_setup);
		_the_boys.push_back(p);
	}
}

void Simulation::order_event_list()
{
	std::sort(_buddy_list.begin(), _buddy_list.end(), reorderer);
}