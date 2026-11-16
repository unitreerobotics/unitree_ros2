cmake_minimum_required(VERSION 3.16)
project(unitree_native_dds_peer C)
# Standalone build permits a different DDS version in the peer process only.
include(../cmake/dds_peer.cmake)
