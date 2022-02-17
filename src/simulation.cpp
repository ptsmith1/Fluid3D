#include "domain.h"
#include "particle.h"
#include "simulation.h"


Simulation::Simulation() :p(particle_mass, particle_radius), m(material_width, material_height, material_depth, p), d(m, zone_size, region_x, region_y, region_z)
{

};

