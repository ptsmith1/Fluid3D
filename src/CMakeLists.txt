cmake_minimum_required(VERSION 3.20)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(Fluid3D VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

get_filename_component(Fluid3D_DIR ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)
set(Fluid3D_SOURCE_DIR $(Fluid3D_DIR)/src)
set(Fluid3D_LIB_DIR $(Fluid3D_DIR)/lib)
set(Fluid3D_TEST_DIR $(Fluid3D_DIR)/test)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

configure_file(Fluid3DConfig.h.in Fluid3DConfig.h)

option(USE_TESTS "Use test library" ON)

if(USE_TESTS)
	add_subdirectory (test)
	list(APPEND EXTRA_LIBS test)
 	list(APPEND EXTRA_INCLUDES "$(Fluid3D_TEST_DIR)")
endif()

add_executable(${PROJECT_NAME} fluid3D.cpp laser.cpp workhorse.cpp domain.cpp particle.cpp simulation.cpp  )

target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_BINARY_DIR}"  ${EXTRA_LIBS})


#gtest
if (NOT googletest) # is googletest defined if already built?
	include(FetchContent)
	FetchContent_Declare(
		googletest
		# Specify the commit you depend on and update it regularly.
		URL https://github.com/google/googletest/archive/refs/heads/main.zip
	)
	# For Windows: Prevent overriding the parent project's compiler/linker settings
	set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
	FetchContent_MakeAvailable(googletest)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           )
