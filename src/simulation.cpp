#include "domain.h"
#include "particle.h"

class Simulation {
private:
	double particle_mass = 1.67e-27;//proton mass
	double particle_radius = 1e-10;
	double material_width = 100e-10;
	double material_height = 100e-10;
	double material_depth = 100e-10;
	double zone_x = 10e-10;
	double zone_y = 10e-10;
	double zone_z = 10e-10;
	double region_x = 100e-10;
	double region_y = 100e-10;
	double region_z = 100e-10;
public:
	Particle p;
	Material m;
	Domain d;
	Simulation() :p(particle_mass, particle_radius) , m(material_width, material_height, material_depth, p) , d(m, zone_x, zone_y, zone_z, region_x, region_y, region_z)
	{

	}
};
