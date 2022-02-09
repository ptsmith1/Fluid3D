#pragma once
#include "particle.h"

class Material {
	//stores information about a material, currently a cuboidal block of one particle type
private:
	double width, height, depth;
	Particle particle_type;
public:
	Material(double w, double h, double d, Particle p);
};

class Domain {
	//zones are small volumes with average properties which effect the particles within them, currently only contain one
	//material type
public:
	double zone_x, zone_y, zone_z; //size of small field effect volumes (will contain maybe 100 particles in dense regions)
	double region_x, region_y, region_z; //total initial size of simulation
	double buffer_region; //a small initial buffer to allow for minor expansion without adding zones
	double laser_expansion_zone; //a large empty region where the laser can expand into before dynamically adding zones
	//auto forbidden_regions; //eg edges of indestructible box

	Material material_type;
	Domain(Material m, double zone_x, double zone_y, double zone_z, double region_x, double region_y, double region_z);
	void calculate_forbidden_regions();
	void calculate_laser_expansion_zone();
};

