// Fluid3D.h : Include file for standard system include files,
// or project specific include files.

#pragma once
#include <iostream>
#include <vector>

class Simulation;

namespace Fluid3D_namespace
{
	class Fluid3D
	{
	private:

	public:
		Fluid3D();
		void save_to_file(const Simulation & sim);
	};
}
