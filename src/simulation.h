#pragma once
#include "domain.h"
#include "particle.h"

static class Simulation {
private:
	double unit_size = 1e-10;
	double material_width = 100e-10;
	double material_height = 100e-10;
	double material_depth = 100e-10;
	double zone_size = 10;
	double region_x = 100;
	double region_y = 100;
	double region_z = 100;
public:
	double dt = 1e-13;
	double run_time = 1e-10;
	int steps = run_time / dt;
	double particle_mass = 1.67e-27;//proton mass
	double particle_radius = 1.5e-10;
	double particle_count = 100;
	Particle p;
	Material m;
	Domain d;
	Simulation();
};