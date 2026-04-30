cmake_minimum_required(VERSION 3.30)

project(storage)

add_library(storage INTERFACE)

target_sources(storage INTERFACE
)

target_include_directories(qd4310 INTERFACE
        ./
)