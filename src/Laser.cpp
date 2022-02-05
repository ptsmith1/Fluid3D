#include "Laser.h"
#include "Fluid3D.h"

using namespace Fluid3D_namespace;

Laser::Laser()
{
	spot_type = "square";
	spot.width = 100e-9;
	spot.height = 100e-9;
	spot.x = spot.y = 0;
	zone_width = 10e-9;
	peak_intensity = 1e20;
	intensity_fwhm = 40e-12;
	peak_time = 100e-12;
	int x_zones = std::floor(spot.width / zone_width);
	int y_zones = std::floor(spot.height / zone_height);
	int z_zones = std::floor(spot.width / zone_width);
}

void Laser::calculate_energy_field()
{

}

void Laser::get_energy_field_at_time()
{
}

void Laser::gaussian_intensity()
{
	for (int time = 0; time < 150; time++)
	{
		intensity = peak_intensity * exp(-4 * log(2) * (pow((time*1e-12 - peak_time), 2) / pow(intensity_fwhm, 2)));
		std::cout << time <<"ps: " << intensity << " W/m^2" << std::endl;
	}
}
