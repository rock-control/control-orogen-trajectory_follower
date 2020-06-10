PROJECT(trajectory_follower)
cmake_minimum_required(VERSION 2.6)
add_definitions(-DENABLE_DEBUG_DRAWINGS)
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++0x" )
SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(trajectory_followerBase)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)