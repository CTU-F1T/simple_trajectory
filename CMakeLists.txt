cmake_minimum_required(VERSION 2.8.3)
project(simple_trajectory)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/dynsimpletrajectory.cfg
)
