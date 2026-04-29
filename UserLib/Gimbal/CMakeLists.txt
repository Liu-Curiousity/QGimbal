cmake_minimum_required(VERSION 3.30)

project(gimbal)

add_library(gimbal INTERFACE)

target_sources(gimbal INTERFACE
        Gimbal.cpp
)

target_include_directories(gimbal INTERFACE
        ./
)