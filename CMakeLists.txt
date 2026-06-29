cmake_minimum_required(VERSION 3.25)

project(ros
	VERSION 0.2.1
	LANGUAGES CXX
	DESCRIPTION
	    "A header-only C++ library that provides safe 
	    and optimized register operations.")

add_library(ros INTERFACE)

if(NOT DEFINED CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

include(cmake/get_cpm.cmake)

cpmaddpackage("gh:boostorg/mp11#boost-1.70.0")

target_link_libraries(ros INTERFACE boost_mp11)

target_sources(
	ros
	INTERFACE FILE_SET
	ros
	TYPE
	HEADERS
	BASE_DIRS
	include
	FILES
	include/ros.hpp
	include/ros/access.hpp
	include/ros/concepts.hpp
	include/ros/error.hpp
	include/ros/filter.hpp
	include/ros/operations.hpp
	include/ros/reg.hpp
	include/ros/eval.hpp
	include/ros/constant.hpp
	include/ros/field.hpp
	include/ros/literals.hpp
	include/ros/reflection.hpp
	include/ros/type_traits.hpp
	include/ros/utils.hpp)

if(PROJECT_IS_TOP_LEVEL)
    add_subdirectory(test)
else()
    add_subdirectory(include)
endif()