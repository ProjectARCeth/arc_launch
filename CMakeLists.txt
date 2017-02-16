cmake_minimum_required(VERSION 2.8.3)
project(arc_launch)


find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
  	roscpp
    catkin_simple REQUIRED
)

catkin_simple(ALL_DEPS_REQUIRED)

cs_install()
cs_export()
