#include "..\Test_header.h"
#include "..\Fluid3D.h"
#include "gtest/gtest.h"
#include "..\event_driven_collisions.h"
TEST(tests, event_driven_tests)
{
	TestClass tester;
	EXPECT_EQ(7, tester.seven);
	EXPECT_EQ(2, tester.two);
	EXPECT_EQ(5, test_function());
}

