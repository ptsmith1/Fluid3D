#pragma once
#include <vector>
#include <random>
#include <string>
#include "simulation.h"



void cycle(Simulation &);
void save_data(Simulation &);
void save_events(Simulation &);
Event create_collision_event(Simulation const&, Particle const& _p1);
Event create_wall_event(Simulation, Particle const& _p);
std::tuple<double, double> overlap(double, double, double, double);
bool get_sign(double x);
void order_event_list(Simulation);
Event compute_event(Simulation &);
std::vector<Event> create_new_events(Simulation const&);
std::vector<Event> create_old_event_list(Simulation const&);
void delete_events(Simulation&, std::vector<Event> const&);
void update_particle_velocities(Simulation);
bool check_dup_event(Simulation const&, Event);
void update_event_times(Simulation &);
void print_event_details(Event const&);
void end_time_update(Simulation&);
void setup_computation(Simulation &);