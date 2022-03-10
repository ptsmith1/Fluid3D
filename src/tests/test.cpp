#include "..\tests\test_header.h"
#include "..\Fluid3D.h"
#include "gtest/gtest.h"
#include "..\event_driven_collisions.h"

Simulation::Simulation(std::string type)
{
	_type = type;
	_particles = 2;
	add_test_particles();
}

void Simulation::add_test_particles()
{
	Particle p1(0);
	p1._pos = { 5e-11,0,0 };
	p1._vel = { -200,0,0 };
	Particle p2(1);
	p2._pos = { -5e-11,0,0 };
	p2._vel = { 200,0,0 };
	_particle_array.push_back(p1);
	_particle_array.push_back(p2);
}

struct Event_driven_oop_tests : testing::Test
{
	Simulation* sim;
	Event e;
	Event e_after;

	Event_driven_oop_tests()
	{
		sim = new Simulation("test");
	 	e = create_wall_event(*sim,sim->_particle_array[1]);
		if (e._null != NULL) { sim->_event_list.push_back(e); }
		//e_after = compute_event(*sim);
	}
};

TEST(tests, event_driven_tests)
{
	TestClass tester;
	EXPECT_EQ(7, tester.seven);
	EXPECT_EQ(2, tester.two);
	EXPECT_EQ(true, get_sign(1));
	EXPECT_EQ(true, get_sign(1e20));
	EXPECT_EQ(true, get_sign(0));
	EXPECT_EQ(false, get_sign(-1));
	EXPECT_EQ(false, get_sign(-1e-20));
}

TEST_F(Event_driven_oop_tests, dunno)
{
	EXPECT_EQ("test", sim->_type);
	EXPECT_EQ(2, sim->_particles);
	EXPECT_EQ(200, sim->_particle_array[1]._vel[0]);
	EXPECT_FALSE(e._null==NULL);
}
