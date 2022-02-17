#include "domain.h"

Domain::Domain(Material m, double z_size, double r_x, double r_y, double r_z) : material_type(m)
{
	zone_size = z_size;
	region_x = r_x;
	region_y = r_y;
	region_z = r_z;
	material_type = m;
}

Material::Material(double w, double h, double d, Particle p) : particle_type(p)
{
	width = w;
	height = h;
	depth = d;
}