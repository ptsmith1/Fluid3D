#include "domain.h"

Domain::Domain(Material m, double z_x, double z_y, double z_z, double r_x, double r_y, double r_z) : material_type(m)
{
	zone_x = z_x;
	zone_y = z_y;
	zone_z = z_z;
	region_x = r_x;
	region_y = r_y;
	region_z = r_z;
}

Material::Material(double w, double h, double d, Particle p) : particle_type(p)
{
	width = w;
	height = h;
	depth = d;
}