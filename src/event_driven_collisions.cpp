#include <iostream>
#include "event_driven_collisions.h"
#include <chrono>
#include <tuple>
#include <algorithm>
#include <fstream>
#include <filesystem>


Event create_collision_event(Simulation const &sim, Particle const& _p1)
{
	//Create each particles event and add to list in order of time
	for (Particle const &_p2 : sim._particle_array)
	{
		double time_since_last_collision = _p1._last_collision_time - _p2._last_collision_time;
		//error -- if p2 has hit something then its pos needs updating, (_p2._pos[0]+ time_since_last_collision* _p2._vel[0]), 
		//as otherwise the algorithm assumes both particles are in the same time reference frame
		//need to rewrite this so each particle has a time it must be advanced for the collision to be valid
		//its sort of working with the current fudge so im making a commit
		if (_p2._id < _p1._id)
		{
			double time_to_collision = 0;
			double t1 = 0, t2 = 0;
			double xdiff = (_p2._pos[0]+ time_since_last_collision* _p2._vel[0]) - _p1._pos[0];
			double vxdiff = _p1._vel[0] - _p2._vel[0];
			t1 = (2 * sim._particle_radius + xdiff) / (vxdiff);
			t2 = (-2 * sim._particle_radius + xdiff) / (vxdiff);
			if (t1 >= 0 || t2 >= 0)
			{
				double t3 = 0, t4 = 0;
				double ydiff = (_p2._pos[1] + time_since_last_collision * _p2._vel[1]) - _p1._pos[1];
				double vydiff = _p1._vel[1] - _p2._vel[1];
				t3 = (2 * sim._particle_radius + ydiff) / vydiff;
				t4 = (-2 * sim._particle_radius + ydiff) / vydiff;
				auto intermediate_interval = overlap(t1, t2, t3, t4);
				if (t3 >= 0 || t4 >= 0 && std::get<0>(intermediate_interval) != -1)
				{
					double t5 = 0, t6 = 0;
					double zdiff = (_p2._pos[2] + time_since_last_collision * _p2._vel[2]) - _p1._pos[2];
					double vzdiff = _p1._vel[2] - _p2._vel[2];
					t5 = (2 * sim._particle_radius + zdiff) / vzdiff;
					t6 = (-2 * sim._particle_radius + zdiff) / vzdiff;
					auto valid_interval = overlap(std::get<0>(intermediate_interval), std::get<1>(intermediate_interval), t5, t6);
					if (std::get<0>(valid_interval) != -1)
					{
						double mid_time = (std::get<0>(intermediate_interval) + std::get<1>(intermediate_interval)) / 2; //valid if no acc
						double x_sep = (_p1._pos[0] + mid_time * _p1._vel[0]) - ((_p2._pos[0] + time_since_last_collision * _p2._vel[0]) + mid_time * _p2._vel[0]);
						double y_sep = (_p1._pos[1] + mid_time * _p1._vel[1]) - ((_p2._pos[1] + time_since_last_collision * _p2._vel[1]) + mid_time * _p2._vel[1]);
						double z_sep = (_p1._pos[2] + mid_time * _p1._vel[2]) - ((_p2._pos[2] + time_since_last_collision * _p2._vel[2]) + mid_time * _p2._vel[2]);
						double sep_at_t = sqrt(x_sep * x_sep + y_sep * y_sep + z_sep * z_sep);
						if (sep_at_t <= 3e-10)
						{
							Event e(_p1, _p2, mid_time, mid_time);
							e._p2._pos = { (_p2._pos[0] + time_since_last_collision * _p2._vel[0]),(_p2._pos[1] + time_since_last_collision * _p2._vel[1]),(_p2._pos[2] + time_since_last_collision * _p2._vel[2]) };//new and fudgey
							if (!check_dup_event(sim, e)) { return e; }
						}
					}
				}
			}
		}
	}
	Event e(NULL);//this is a fudge, there must be a better way
	return e;
}

Event create_wall_event(Simulation sim, Particle const& _p)
{
	double time_to_x_wall =0, time_to_y_wall=0, time_to_z_wall=0;
	std::vector<double> wall_vector = { 0,0,0 };
	if (_p._vel[0]>0)
	{
		time_to_x_wall = (sim._box._x_boundary - _p._pos[0]) / _p._vel[0];
	}
	else if (_p._vel[0] < 0)
	{
		time_to_x_wall = (-sim._box._x_boundary - _p._pos[0]) / _p._vel[0];
	}
	if (_p._vel[1] > 0)
	{
		time_to_y_wall = (sim._box._y_boundary - _p._pos[1]) / _p._vel[1];
	}
	else if (_p._vel[1] < 0)
	{
		time_to_y_wall = (-sim._box._y_boundary - _p._pos[1]) / _p._vel[1];
	}
	if (_p._vel[2] > 0)
	{
		time_to_z_wall = (sim._box._z_boundary - _p._pos[2]) / _p._vel[2];
	}
	else if (_p._vel[2] < 0)
	{
		time_to_z_wall = (-sim._box._y_boundary - _p._pos[2]) / _p._vel[2];
	}
	std::vector<double> wall_times = { time_to_x_wall,time_to_y_wall,time_to_z_wall };
	std::vector<double>::iterator result = std::min_element(wall_times.begin(), wall_times.end());
	int axis_hit = std::distance(wall_times.begin(), result);
	Event e(_p, axis_hit, result[0],result[0]);
	return e;
}

std::tuple<double, double> overlap(double t1, double t2, double t3, double t4)
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

bool get_sign(double x)
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

Event compute_event(Simulation &sim)
{
	//for test purposes create a new event with values from popped event, update this event, then return it 
	//and set the event pointed to by popped event equal to this new event
	//this currently updates vel and pos of popped event particles as well as actual particles in the boys
	//dont currently need to update properties of popped event but may need to at some point
	Event e = *sim._popped_event;
	if (sim._popped_event->_type == "particle")
	{
		for (int i = 0; i < 3; i++)
		{
			e._p1._pos[i] = e._p1._pos[i] + e._original_time_to_collision * e._p1._vel[i];
			e._p2._pos[i] = e._p2._pos[i] + e._original_time_to_collision * e._p2._vel[i];
		}
		double x_sep = e._p1._pos[0] - e._p2._pos[0];
		double y_sep = e._p1._pos[1] - e._p2._pos[1];
		double z_sep = e._p1._pos[2] - e._p2._pos[2];
		double vx_sep = e._p1._vel[0] - e._p2._vel[0];
		double vy_sep = e._p1._vel[1] - e._p2._vel[1];
		double vz_sep = e._p1._vel[2] - e._p2._vel[2];

		e._p1._vel[0] += -x_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * sim._particle_radius * sim._particle_radius);
		e._p2._vel[0] += x_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * sim._particle_radius * sim._particle_radius);

		e._p1._vel[1] += -y_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * sim._particle_radius * sim._particle_radius);
		e._p2._vel[1] += y_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * sim._particle_radius * sim._particle_radius);

		e._p1._vel[2] += -z_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * sim._particle_radius * sim._particle_radius);
		e._p2._vel[2] += z_sep * (x_sep * vx_sep + y_sep * vy_sep + z_sep * vz_sep) / (4 * sim._particle_radius * sim._particle_radius);

		//updates particles in the boys
		sim._particle_array[e._p1._id]._vel = e._p1._vel;
		sim._particle_array[e._p1._id]._pos = e._p1._pos;
		sim._particle_array[e._p2._id]._vel = e._p2._vel;
		sim._particle_array[e._p2._id]._pos = e._p2._pos;
		//update timings
		e._p1._last_collision_time = sim._sim_time; //time of this collision
		e._p2._last_collision_time = sim._sim_time;
		sim._particle_array[e._p1._id]._last_collision_time = sim._sim_time;
		sim._particle_array[e._p2._id]._last_collision_time = sim._sim_time;
	}

	else if (e._type == "wall")
	{
		for (int i = 0; i < 3; i++)
		{
			e._p1._pos[i] = e._p1._pos[i] + e._original_time_to_collision * e._p1._vel[i];
		}
		e._p1._vel[e._axis_hit] *= -1;
		sim._particle_array[e._p1._id]._vel = e._p1._vel;
		sim._particle_array[e._p1._id]._pos = e._p1._pos;
		e._p1._last_collision_time = sim._sim_time; //time of this collision
		sim._particle_array[e._p1._id]._last_collision_time = sim._sim_time;
	}
	e._sim_time = sim._sim_time;
	sim._collisions++;
	return e;
}

std::vector<Event> create_new_events(Simulation const & sim)
{
	std::vector<Particle> affected_particles = { sim._popped_event->_p1 };
	if (sim._popped_event->_type == "particle")
	{
		affected_particles.push_back(sim._popped_event->_p2);
	}
	//delete old events
	for (Event e : sim._event_list)
	{
		if (e._type == "particle")
		{
			if (sim._popped_event->_p1._id == e._p1._id)
			{
				//pop event
				affected_particles.push_back(e._p2);
			}

			if (sim._popped_event->_p1._id == e._p2._id)
			{
				//pop event
				affected_particles.push_back(e._p1);
			}
		}
	}

	std::vector<Event> new_events;
	for (Particle const& p : affected_particles)
	{
		Event new_e = create_collision_event(sim, p);
		if (new_e._null != NULL) { new_events.push_back(new_e); std::cout << "Event added: " << new_e._id << std::endl; }
		new_e = create_wall_event(sim, p);
		if (new_e._null != NULL) { new_events.push_back(new_e); std::cout << "Event added: " << new_e._id << std::endl; }
	}
	return new_events;
}

std::vector<Event> create_old_event_list(Simulation const & sim)
{
	std::vector<Event> popped_events = { };

	for (Event e : sim._event_list)
	{
		if (e._type == "wall")
		{
			if (sim._popped_event->_p1._id == e._p1._id)
			{
				popped_events.push_back(e);
			}

			if (sim._popped_event->_type == "particle")
			{
				if (sim._popped_event->_p2._id == e._p1._id)
				{
					popped_events.push_back(e);
				}
			}
		}

		if (e._type == "particle")
		{
			if (sim._popped_event->_p1._id == e._p1._id)
			{
				popped_events.push_back(e);
			}

			if (sim._popped_event->_p1._id == e._p2._id)
			{
				popped_events.push_back(e);
			}

			if (sim._popped_event->_type == "particle")
			{
				if (sim._popped_event->_p2._id == e._p1._id || sim._popped_event->_p2._id == e._p2._id)
				{
					popped_events.push_back(e);
				}

			}
		}
	}
	return popped_events;
}

void delete_events(Simulation & sim, std::vector<Event> const& popped_events)
{
	for (auto e : popped_events)
	{
		std::string id = e._id;
		auto it = std::find_if(sim._event_list.begin(), sim._event_list.end(), [&id](const Event& obj) {return obj._id == id; });
		if (it < sim._event_list.end())
		{
			sim._event_list.erase(it);
		}
	}
}

bool check_dup_event(Simulation const & sim, Event new_event)
{ //return true if id of new event matches the id/reverse id of event in event list
	for (Event e : sim._event_list)
	{
		std::string reversed(new_event._id.rbegin(), new_event._id.rend());
		if (e._id == new_event._id || e._id == reversed) {return true;}
	}
	return false;
}

void update_event_times(Simulation &sim)
{
	for (Event& e : sim._event_list)
	{
		e._current_time_to_collision = e._current_time_to_collision - sim._popped_event->_current_time_to_collision;
	}
}

void print_event_details(Event const & e)
{
	std::cout << "ID: " << e._id << std::endl;
	std::cout << "Particle " << e._p1._id << " position: (" << e._p1._pos[0] << ", " << e._p1._pos[1] << ", " << e._p1._pos[2] << ") velocity: (" << e._p1._vel[0] << ", " << e._p1._vel[1] << ", " << e._p1._vel[2] << ")" << std::endl;
	std::cout << "Particle " << e._p2._id << " position : (" << e._p2._pos[0] << ", " << e._p2._pos[1] << ", " << e._p2._pos[2] << ") velocity : (" << e._p2._vel[0] << ", " << e._p2._vel[1] << ", " << e._p2._vel[2] << ")" << std::endl;
}

void save_events(Simulation &sim)
{
	std::reverse(sim._processed_events.begin(), sim._processed_events.end());
	double save_time = sim._time_save_interval;
	int it = 0;
	while (save_time < sim._run_time)
	{
		it++;
		int offset = (sim._particles * 6) * (it-1);
		for (Particle const &p : sim._particle_array)
		{
			sim._particle_data.push_back(sim._particle_data[offset + (p._id * 6)] + sim._time_save_interval * sim._particle_data[offset + (p._id * 6) + 3]);
			sim._particle_data.push_back(sim._particle_data[offset + (p._id * 6)+1] + sim._time_save_interval * sim._particle_data[offset + (p._id * 6) + 4]);
			sim._particle_data.push_back(sim._particle_data[offset + (p._id * 6)+2] + sim._time_save_interval * sim._particle_data[offset + (p._id * 6) + 5]);
			sim._particle_data.push_back(sim._particle_data[offset + (p._id * 6)+3]);
			sim._particle_data.push_back(sim._particle_data[offset + (p._id * 6)+4]);
			sim._particle_data.push_back(sim._particle_data[offset + (p._id * 6)+5]);
		}
		//modifies the data that has just been added due to an event having occurred
		offset = (sim._particles * 6) * (it);
		for (Event const &e : sim._processed_events)
		{
			if (save_time>e._sim_time && (save_time - e._sim_time) <= sim._time_save_interval)
			{
				//if an event time has been reached, then the particles in the event have their saved pos/vel updated in the next save 
				//timestep. This overwrites the previous data which assumed no collision had occurred. 
				//pass of event added
				//there is a bit of a complex issue here where hopping to the timestep after the one where an event is added to
				//the output data, updates a particles positio with a full timestep of its new velocity when part of that 
				//timestep should use the old velocity before the event
				sim._particle_data[offset + (e._p1._id * 6)] = e._p1._pos[0] + (e._p1._vel[0] * (save_time - e._sim_time));
				sim._particle_data[offset + (e._p1._id * 6) + 1] = e._p1._pos[1] + (e._p1._vel[1] * (save_time - e._sim_time));
				sim._particle_data[offset + (e._p1._id * 6) + 2] = e._p1._pos[2] + (e._p1._vel[2] * (save_time - e._sim_time));
				sim._particle_data[offset + (e._p1._id * 6) + 3] = e._p1._vel[0];
				sim._particle_data[offset + (e._p1._id * 6) + 4] = e._p1._vel[1];
				sim._particle_data[offset + (e._p1._id * 6) + 5] = e._p1._vel[2];
				if (e._type != "wall")
				{
					sim._particle_data[offset + (e._p2._id * 6)] = e._p2._pos[0] + (e._p2._vel[0] * (save_time - e._sim_time));
					sim._particle_data[offset + (e._p2._id * 6) + 1] = e._p2._pos[1] + (e._p2._vel[1] * (save_time - e._sim_time));
					sim._particle_data[offset + (e._p2._id * 6) + 2] = e._p2._pos[2] + (e._p2._vel[2] * (save_time - e._sim_time));
					sim._particle_data[offset + (e._p2._id * 6) + 3] = e._p2._vel[0];
					sim._particle_data[offset + (e._p2._id * 6) + 4] = e._p2._vel[1];
					sim._particle_data[offset + (e._p2._id * 6) + 5] = e._p2._vel[2];
				}
				
				
			}
		}
		save_time += sim._time_save_interval;
	}
}

void save_data(Simulation & sim)
{
	//saves current particle positions and velocities to array
	for (Particle p : sim._particle_array)
	{
		sim._particle_data.push_back(p._pos[0]);
		sim._particle_data.push_back(p._pos[1]);
		sim._particle_data.push_back(p._pos[2]);
		sim._particle_data.push_back(p._vel[0]);
		sim._particle_data.push_back(p._vel[1]);
		sim._particle_data.push_back(p._vel[2]);
	}
}

void end_time_update(Simulation & sim)
{
	//im not sure if this is still necessary
	for (Particle& p : sim._particle_array)
	{
		double travel_time = sim._sim_time - p._last_collision_time;
		for (int i = 0; i < 3; i++)
		{
			p._pos[i] += p._vel[i] * travel_time;
		}
	}
}

void cycle(Simulation& sim)
{
	while (sim._sim_time < sim._run_time && !sim._event_list.empty())
	{
		//compute first event, invalidate some events, calculate new events for these particles, compute next event
		std::cout << "Before collision:" << std::endl;
		sim._popped_event = &sim._event_list.back();
		sim._sim_time += sim._popped_event->_current_time_to_collision;
		print_event_details(*sim._popped_event);
		std::cout << "This timestep = " << sim._popped_event->_current_time_to_collision << " Total time = " << sim._sim_time << std::endl;
		*sim._popped_event = compute_event(sim);
		update_event_times(sim);
		sim._processed_events.push_back(*sim._popped_event);
		std::cout << "After collision:" << std::endl;
		print_event_details(*sim._popped_event);
		std::cout << std::endl;
		std::vector<Event> new_events = create_new_events(sim);
		std::vector<Event> popped_events = create_old_event_list(sim);
		//end of cycle: add old event to processed events, pop old event, add new event
		delete_events(sim, popped_events);
		sim._event_list.insert(std::end(sim._event_list), std::begin(new_events), std::end(new_events));
		sim.order_event_list();
	}
}

void setup_computation(Simulation &sim)
{
	//Create each particles event and add to list in order of time
	for (Particle const & _p1 : sim._particle_array)
	{
		Event e = create_collision_event(sim, _p1);
		if (e._null!=NULL) { sim._event_list.push_back(e); }
		e = create_wall_event(sim, _p1);
		if (e._null != NULL) { sim._event_list.push_back(e); }
	}
	sim.order_event_list();
	save_data(sim);
	cycle(sim);
	end_time_update(sim);
	save_events(sim);
	save_data(sim);
}
