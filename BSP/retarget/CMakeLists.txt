cmake_minimum_required(VERSION 3.30)

project(retarget)

add_library(retarget INTERFACE)

target_sources(retarget INTERFACE retarget.cpp printf.c)

target_include_directories(retarget INTERFACE
        ./
)