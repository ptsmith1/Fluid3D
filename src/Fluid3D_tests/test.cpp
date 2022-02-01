#include "..\Test_header.h"
#include "..\Fluid3D.h"
#include "gtest/gtest.h"

TEST(Tests, Test1)
{
	TestClass tester;
	EXPECT_EQ(7, tester.seven);
	EXPECT_EQ(2, tester.two);
}

