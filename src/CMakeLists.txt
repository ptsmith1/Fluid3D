cmake_minimum_required(VERSION 3.20)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
enable_testing()

project(Fluid3D VERSION 1.0)

get_filename_component(Fluid3D_DIR ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)
set(Fluid3D_SOURCE_DIR $(Fluid3D_DIR)/../src)
set(Fluid3D_LIB_DIR $(Fluid3D_DIR)/library)
set(Fluid3D_TEST_DIR $(Fluid3D_DIR)/test)
set(Fluid3D_PYTHON_DIR $(Fluid3D_DIR)/python)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

set(Headers
fluid3D.h laser.h workhorse.h domain.h particle.h
simulation.h event_driven_collisions.h
)
set(Sources
fluid3D.cpp laser.cpp workhorse.cpp domain.cpp particle.cpp
simulation.cpp event_driven_collisions.cpp
)

add_executable(${PROJECT_NAME} ${Sources})
add_library(${PROJECT_NAME}_lib STATIC ${Sources} ${Headers}  ) #change this to add_executable if you want to compile as executables

configure_file(Fluid3DConfig.h.in Fluid3DConfig.h)
################################################################################################################
option(USE_TESTS "Use test library" ON)

if(USE_TESTS)
	add_subdirectory (tests)
	list(APPEND EXTRA_LIBS tests)
 	list(APPEND EXTRA_INCLUDES "$(Fluid3D_TEST_DIR)")
endif()
#################################################################################################################
#add_subdirectory(python)
#list(APPEND EXTRA_LIBS python)
#################################################################################################################
add_subdirectory(library)
list(APPEND EXTRA_LIBS library)
#################################################################################################################
#Download submodules from github
find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${Fluid3D_DIR}/.git")
	option(GIT_SUBMODULE "Check submodules during build" ON)
	if(GIT_SUBMODULE)
		message(STATUS "Submodule update")
		execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
						WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
						RESULT_VARIABLE GIT_SUBMOD_RESULT)
		if(NOT GIT_SUBMOD_RESULT EQUAL "0")
			message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
		endif()
	endif()
endif()
#check submodules
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/external/glfw/CMakeLists.txt")
	message(FATAL_ERROR "The glfw submodules was not downloaded! GIT_SUBMODULE was turned off or failed.")
endif()
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/external/gtest/CMakeLists.txt")
	message(FATAL_ERROR "The gtest submodules was not downloaded! GIT_SUBMODULE was turned off or failed.")
endif()

add_subdirectory(external/glfw)
add_subdirectory(external/gtest)
list(APPEND EXTRA_LIBS external/glfw/include)
list(APPEND EXTRA_LIBS external/gtest/include)
#################################################################################################################
target_include_directories(${PROJECT_NAME} PUBLIC ${EXTRA_LIBS})
target_link_directories(${PROJECT_NAME} PUBLIC library PUBLIC external/glfw/src PUBLIC external/gtest/src) #paths from build folder
target_link_libraries(${PROJECT_NAME} adder glfw gtest) #looks for adder.lib, need to do this for each library file in the library folder
#################################################################################################################

