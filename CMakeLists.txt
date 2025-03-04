cmake_minimum_required(VERSION 3.25)

project(ros
	VERSION 0.0.1
	LANGUAGES CXX
	DESCRIPTION
	    "A header-only C++ library that provides safe 
	    and optimized register operations.")

add_library(ros INTERFACE)

target_compile_features(ros INTERFACE cxx_std_23)

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
	include/ros/apply.hpp
	include/ros/constant.hpp
	include/ros/field.hpp
	include/ros/literals.hpp
	include/ros/reflection.hpp
	include/ros/type_traits.hpp)

if(PROJECT_IS_TOP_LEVEL)
    # add_docs(docs)
    # clang_tidy_interface(ros)
    # add_subdirectory(test)
else()
    add_subdirectory(include)
endif()


# find include -regex '.*\(cpp\|hpp\)' -exec /usr/bin/clang-format-18 -i {} \;