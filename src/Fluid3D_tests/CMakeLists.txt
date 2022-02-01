cmake_minimum_required(VERSION 3.2)

project(Fluid3D_tests VERSION 1.0)

# link against gtest or gtest_main as needed. Eg
add_executable(${PROJECT_NAME} test.cpp)
target_link_libraries(${PROJECT_NAME}
	gtest_main
)

add_test(NAME MatrixVector_Test COMMAND ${PROJECT_NAME})

# post build command
add_custom_command(
     TARGET ${PROJECT_NAME}  
     COMMENT "Run tests"
     POST_BUILD 
     COMMAND ${PROJECT_NAME}
)