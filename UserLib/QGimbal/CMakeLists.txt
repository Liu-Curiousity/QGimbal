cmake_minimum_required(VERSION 3.30)

project(qgimbal)

add_library(qgimbal INTERFACE)

target_sources(qgimbal INTERFACE QGimbal.cpp)

target_include_directories(qgimbal INTERFACE
        ./
)