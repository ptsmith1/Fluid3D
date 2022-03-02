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
	_pos_range = std::uniform_real_distribution<double>{ minp, maxp };
	_vel_range = std::uniform_real_distribution<double>{ minv, maxv };
}

New_Particle::New_Particle(int _id)
{
	this->_id = _id;
	_pos = std::vector<double>(3);
	_vel = std::vector<double>(3);
}

void New_Particle::init_vectors(Random_setup& _random_setup)
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

Event::Event(New_Particle p1, New_Particle p2, double collision_time, double time_of_collision) :_p1(p1), _p2(p2), _collision_time(collision_time), _time_of_collision(time_of_collision)
{
	//collision event
	_type = "particle";
	_id = std::to_string(_p1._id) + "-" + std::to_string(_p2._id);
}
Event::Event(New_Particle p, int axis_hit, double collision_time, double time_of_collision) : _p1(p), _axis_hit(axis_hit), _p2(NULL), _collision_time(collision_time), _time_of_collision(time_of_collision)
{
	//wall event
	_type = "wall";
	_id = std::to_string(_p1._id) + "-wall";
}

void Event::update_particles(int i)
{
	_p1 = i;
	_p2 = i;
}

Compute::Compute() : _random_setup(300, -300, _box_size, -_box_size), _popped_event(0,0,0,0)
{
	_time = 0;
	add_particles();
	//Create each particles event and add to list in order of time
	for (New_Particle _p1 : _the_boys)
	{
		create_collision_event(_p1);
		create_wall_event(_p1);
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
	for (int i = 0; i < _particles; i++)
	{
		New_Particle p(i);
		p.init_vectors(_random_setup);
		_the_boys.push_back(p);
	}
}

void Compute::create_collision_event(New_Particle _p1)
{
	//Create each particles event and add to list in order of time
	for (New_Particle _p2 : _the_boys)
	{
		if (_p2._id < _p1._id)
		{
			double time_to_collision = 0;
			double t1 = 0, t2 = 0;
			double xdiff = _p1._pos[0] - _p2._pos[0];
			double vxdiff = _p1._vel[0] - _p2._vel[0];
			t1 = (2 * _particle_radius - xdiff) / vxdiff;
			t2 = (-2 * _particle_radius - xdiff) / vxdiff;
			if (t1 >= 0 && t2 >= 0)
			{
				double t3 = 0, t4 = 0;
				double ydiff = _p1._pos[1] - _p2._pos[1];
				double vydiff = _p1._vel[1] - _p2._vel[1];
				t3 = (2 * _particle_radius - ydiff) / vydiff;
				t4 = (-2 * _particle_radius - ydiff) / vydiff;
				auto intermediate_interval = overlap(t1, t2, t3, t4);
				if (t3 >= 0 && t4 >= 0 && std::get<0>(intermediate_interval) != -1)
				{
					double t5 = 0, t6 = 0;
					double zdiff = _p1._pos[2] - _p2._pos[2];
					double vzdiff = _p1._vel[2] - _p2._vel[2];
					t5 = (2 * _particle_radius - zdiff) / vzdiff;
					t6 = (-2 * _particle_radius - zdiff) / vzdiff;
					auto valid_interval = overlap(std::get<0>(intermediate_interval), std::get<1>(intermediate_interval), t5, t6);
					if (std::get<0>(valid_interval) != -1)
					{
						double mid_time = (std::get<0>(intermediate_interval) + std::get<1>(intermediate_interval)) / 2; //valid if no acc
						double x_sep = (_p1._pos[0] + mid_time * _p1._vel[0]) - (_p2._pos[0] + mid_time * _p2._vel[0]);
						double y_sep = (_p1._pos[1] + mid_time * _p1._vel[1]) - (_p2._pos[1] + mid_time * _p2._vel[1]);
						double z_sep = (_p1._pos[2] + mid_time * _p1._vel[2]) - (_p2._pos[2] + mid_time * _p2._vel[2]);
						double sep_at_t = sqrt(x_sep * x_sep + y_sep * y_sep + z_sep * z_sep);
						if (sep_at_t <= 3e-10)
						{
							Event e(_p1, _p2, mid_time, mid_time);
							if (!check_dup_event(e)) { _buddy_list.push_back(e); }
							std::cout << "Created collision event ID: " << e._id << std::endl;
						}
					}
				}
			}
		}
	}
}

void Compute::create_wall_event(New_Particle _p)
{
	double time_to_x_wall =0, time_to_y_wall=0, time_to_z_wall=0;
	std::vector<double> wall_vector = { 0,0,0 };
	if (_p._vel[0]>0)
	{
		time_to_x_wall = (_box._x_boundary - _p._pos[0]) / _p._vel[0];
	}
	else if (_p._vel[0] < 0)
	{
		time_to_x_wall = (-_box._x_boundary - _p._pos[0]) / _p._vel[0];
	}
	if (_p._vel[1] > 0)
	{
		time_to_y_wall = (_box._y_boundary - _p._pos[1]) / _p._vel[1];
	}
	else if (_p._vel[1] < 0)
	{
		time_to_y_wall = (-_box._y_boundary - _p._pos[1]) / _p._vel[1];
	}
	if (_p._vel[2] > 0)
	{
		time_to_z_wall = (_box._z_boundary - _p._pos[2]) / _p._vel[2];
	}
	else if (_p._vel[2] < 0)
	{
		time_to_z_wall = (-_box._y_boundary - _p._pos[2]) / _p._vel[2];
	}
	std::vector<double> wall_times = { time_to_x_wall,time_to_y_wall,time_to_z_wall };
	std::vector<double>::iterator result = std::min_element(wall_times.begin(), wall_times.end());
	int axis_hit = std::distance(wall_times.begin(), result);
	Event e(_p, axis_hit, result[0],result[0]);
	if (!check_dup_event(e)) { _buddy_list.push_back(e); }
	std::cout << "Created wall event ID: " << e._id << std::endl;
}

std::tuple<double, double> Compute::overlap(double t1, double t2, double t3, double t4)
{
	//takes two time periods t1-t2 and t3-t4 and returns any overlap time period
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
		return (e1._collision_time > e2._collision_time); //change > to order vector back to front
	}
} reorderer;

void Compute::order_event_list()
{
	std::sort(_buddy_list.begin(), _buddy_list.end(), reorderer);
}

void Compute::cycle()
{
	while (_time < _run_time && !_buddy_list.empty())
	{
		//compute first event, invalidate some events, calculate new events for these particles, compute next event
		std::cout << "Before collision:" << std::endl;
		_popped_event = _buddy_list.back();
		print_event_details(_popped_event);
		compute_event();
		update_event_times();
		update_event_list();
		order_event_list();
		std::cout << "After collision:" << std::endl;
		print_event_details(_popped_event);
		_time += _popped_event._collision_time;
		std::cout << "This timestep = " << _popped_event._collision_time << " Total time = " << _time << std::endl;
		std::cout << std::endl;
		_processed_events.push_back(_popped_event);
	}
	end_time_update();
}

void Compute::compute_event()
{
	//this currently updates vel and pos of popped event particles as well as actual particles in the boys
	//dont currently need to update properties of popped event but may need to at some point
	_popped_event._time = _time + _popped_event._collision_time;
	if (_popped_event._type == "collision")
	{
		for (int i = 0; i < 3; i++)
		{
			_popped_event._p1._pos[i] = _the_boys[_popped_event._p1._id]._pos[i] + _popped_event._time_of_collision * _popped_event._p1._vel[i];
			_popped_event._p2._pos[i] = _the_boys[_popped_event._p2._id]._pos[i] + _popped_event._time_of_collision * _popped_event._p2._vel[i];
		}
		double x_sep = _popped_event._p1._pos[0] - _popped_event._p2._pos[0];
		double y_sep = _popped_event._p1._pos[1] - _popped_event._p2._pos[1];
		double z_sep = _popped_event._p1._pos[2] - _popped_event._p2._pos[2];
		double vx_sep = _popped_event._p1._vel[0] - _popped_event._p2._vel[0];
		double vy_sep = _popped_event._p1._vel[1] - _popped_event._p2._vel[1];
		double vz_sep = _popped_event._p1._vel[2] - _popped_event._p2._vel[2];

		_popped_event._p1._vel[0] += -x_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * _particle_radius * _particle_radius);
		_popped_event._p2._vel[0] += x_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * _particle_radius * _particle_radius);

		_popped_event._p1._vel[1] += -y_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * _particle_radius * _particle_radius);
		_popped_event._p2._vel[1] += y_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * _particle_radius * _particle_radius);

		_popped_event._p1._vel[2] += -z_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * _particle_radius * _particle_radius);
		_popped_event._p2._vel[2] += z_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * _particle_radius * _particle_radius);

		//updates particles in the boys
		_the_boys[_popped_event._p1._id]._vel = _popped_event._p1._vel;
		_the_boys[_popped_event._p1._id]._pos = _popped_event._p1._pos;
		_the_boys[_popped_event._p2._id]._vel = _popped_event._p2._vel;
		_the_boys[_popped_event._p2._id]._pos = _popped_event._p2._pos;
		_popped_event._p1._last_collision_time = _time + _popped_event._collision_time; //time of this collision
		_popped_event._p2._last_collision_time = _time + _popped_event._collision_time;
		_the_boys[_popped_event._p1._id]._last_collision_time = _popped_event._p1._last_collision_time;
		_the_boys[_popped_event._p2._id]._last_collision_time = _popped_event._p2._last_collision_time;
	}

	else if (_popped_event._type == "wall")
	{
		for (int i = 0; i < 3; i++)
		{
			_popped_event._p1._pos[i] = _the_boys[_popped_event._p1._id]._pos[i] + _popped_event._time_of_collision * _popped_event._p1._vel[i];
		}
		_popped_event._p1._vel[_popped_event._axis_hit] *= -1;
		_the_boys[_popped_event._p1._id]._vel = _popped_event._p1._vel;
		_the_boys[_popped_event._p1._id]._pos = _popped_event._p1._pos;
		_popped_event._p1._last_collision_time = _time + _popped_event._collision_time; //time of this collision
		_the_boys[_popped_event._p1._id]._last_collision_time = _popped_event._p1._last_collision_time;
	}

	_buddy_list.pop_back();
	_collisions++;
}

void Compute::update_event_list()
{
	//create_collision_event(_popped_event._p1);
	for (Event &e : _buddy_list)
	{
		if (_popped_event._p1._id == e._p1._id || (_popped_event._p2._id == e._p1._id && e._type != "wall") || _popped_event._p1._id == e._p2._id || (_popped_event._p2._id == e._p2._id && e._type != "wall"))
		{
			//new events for particles that would have hit the collided particles had they not collided
			e._collision_time = 999;
			create_collision_event(e._p1);
			create_wall_event(e._p1);
			if (_popped_event._type == "particle")
			{
				create_collision_event(e._p2);
				create_wall_event(e._p2);
			}
		}
	}

	create_wall_event(_popped_event._p1);
	if (_popped_event._type == "particle")
	{
		create_collision_event(_popped_event._p2);
		create_wall_event(_popped_event._p2);
	}

}

bool Compute::check_dup_event(Event new_event)
{ //return true if id of new event matches the id/reverse id of event in event list
	for (Event &e : _buddy_list)
	{
		std::string reversed(new_event._id.rbegin(), new_event._id.rend());
		if (e._id == new_event._id || e._id == reversed) {return true;}
	}
	return false;
}

void Compute::update_event_times()
{
	for (Event& e : _buddy_list)
	{
		e._collision_time = e._collision_time - _popped_event._collision_time;
	}
}

void Compute::print_event_details(Event e)
{
	std::cout << "ID: " << e._id << std::endl;
	std::cout << "Particle " << e._p1._id << " position: (" << e._p1._pos[0] << ", " << e._p1._pos[1] << ", " << e._p1._pos[2] << ") velocity: (" << e._p1._vel[0] << ", " << e._p1._vel[1] << ", " << e._p1._vel[2] << ")" << std::endl;
	std::cout << "Particle " << e._p2._id << " position : (" << e._p2._pos[0] << ", " << e._p2._pos[1] << ", " << e._p2._pos[2] << ") velocity : (" << e._p2._vel[0] << ", " << e._p2._vel[1] << ", " << e._p2._vel[2] << ")" << std::endl;
}

void Compute::end_time_update()
{
	for (New_Particle& p : _the_boys)
	{
		double travel_time = _time - p._last_collision_time;
		for (int i = 0; i < 3; i++)
		{
			p._pos[i] += p._vel[i] * travel_time;
		}
	}
}

void Compute::save_events()
{
	std::reverse(_processed_events.begin(), _processed_events.end());
	double save_time = 0;
	int it = 0;
	while (save_time < _run_time)
	{
		it++;
		save_time += _time_save_interval;
		int offset = (_particles * 6) * (it-1);
		for (New_Particle &p : _the_boys)
		{
			_particle_data.push_back(_particle_data[offset + (p._id * 6)] + _time_save_interval * _particle_data[offset + (p._id * 6) + 3]);
			_particle_data.push_back(_particle_data[offset + (p._id * 6)+1] + _time_save_interval * _particle_data[offset + (p._id * 6) + 4]);
			_particle_data.push_back(_particle_data[offset + (p._id * 6)+2] + _time_save_interval * _particle_data[offset + (p._id * 6) + 5]);
			_particle_data.push_back(_particle_data[offset + (p._id * 6)+3]);
			_particle_data.push_back(_particle_data[offset + (p._id * 6)+4]);
			_particle_data.push_back(_particle_data[offset + (p._id * 6)+5]);
		}
		//modifies the data that has just been added due to an event having occurred
		offset = (_particles * 6) * (it);
		for (Event &e : _processed_events)
		{
			if (save_time>e._time && (save_time - e._time) <= _time_save_interval)
			{
				//if an event time has been reached, then the particles in the event have their saved pos/vel updated in the next save 
				//timestep. This overwrites the previous data which assumed no collision had occurred. 
				//pass of event added
				//there is a bit of a complex issue here where hopping to the timestep after the one where an event is added to
				//the output data, updates a particles positio with a full timestep of its new velocity when part of that 
				//timestep should use the old velocity before the event
				_particle_data[offset + (e._p1._id * 6)] = e._p1._pos[0] + (e._p1._vel[0] * (save_time - e._time));
				_particle_data[offset + (e._p1._id * 6) + 1] = e._p1._pos[1] + (e._p1._vel[1] * (save_time - e._time));
				_particle_data[offset + (e._p1._id * 6) + 2] = e._p1._pos[2] + (e._p1._vel[2] * (save_time - e._time));
				_particle_data[offset + (e._p1._id * 6) + 3] = e._p1._vel[0];
				_particle_data[offset + (e._p1._id * 6) + 4] = e._p1._vel[1];
				_particle_data[offset + (e._p1._id * 6) + 5] = e._p1._vel[2];
				if (e._type != "wall")
				{
					_particle_data[offset + (e._p2._id * 6)] = e._p2._pos[0] + (e._p2._vel[0] * (save_time - e._time));
					_particle_data[offset + (e._p2._id * 6) + 1] = e._p2._pos[1] + (e._p2._vel[1] * (save_time - e._time));
					_particle_data[offset + (e._p2._id * 6) + 2] = e._p2._pos[2] + (e._p2._vel[2] * (save_time - e._time));
					_particle_data[offset + (e._p2._id * 6) + 3] = e._p2._vel[0];
					_particle_data[offset + (e._p2._id * 6) + 4] = e._p2._vel[1];
					_particle_data[offset + (e._p2._id * 6) + 5] = e._p2._vel[2];
				}
				
				
			}
		}
	}
}

void Compute::save_data()
{
	//saves current particle positions and velocities to array
	for (New_Particle p : _the_boys)
	{
		_particle_data.push_back(p._pos[0]);
		_particle_data.push_back(p._pos[1]);
		_particle_data.push_back(p._pos[2]);
		_particle_data.push_back(p._vel[0]);
		_particle_data.push_back(p._vel[1]);
		_particle_data.push_back(p._vel[2]);
	}
}

void Compute::data_to_csv()
{
	std::ofstream f;
	std::filesystem::path filePath = std::filesystem::current_path();
	filePath.append("fluid_data.csv");
	f.open(filePath.string());
	f << "sim_size," << "box_size," << "collisions," << "timesteps" << "\n";
	f << _particles << "," << _box_size << "," << _collisions << "," << (_run_time / _time_save_interval) + 1 << "\n";
	f << "Timestep";
	for (int j = 0; j < _particles; j += 1)
	{
		f << ",x" << j << ",y" << j << ",z" << j << ",vx" << j << ",vy" << j << ",vz" << j;
	}
	f << "\n";

	for (int it_col = 0; it_col < (_run_time / _time_save_interval) + 1; it_col++)
	{
		f << it_col;
		for (int it_row = 0; it_row < (_particles * 6); it_row++)
		{
			f << "," << _particle_data[it_row + _particles * 6 * it_col];
		}
		f << "\n";
	}
	f.close();
}

int test_function()
{
	return 5;
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
