#pragma once
#include <string>
/*
This class creates a new laser source(s), it works out how the laser will interact with 
the material, the actual absorption of the laser is done elsewhere.
The laser will initialy be a perfect cylinder with varying intensity at different times
*/

class Laser
{
private:
	//user inputs
	std::string spot_type;
	double spot_radius;
	double pulse_width;
	double intensity_fwhm;
	double wavelength;
	double peak_time;
	double zone_width;
	double zone_height;
	double zone_depth;
	double peak_intensity;
	double intensity;
	//
	struct spot_dims
	{
		double x;
		double y;
		double width;
		double height;
	} spot;
public:
	Laser();
	void calculate_energy_field();
	void get_energy_field_at_time();
	void gaussian_intensity();
};