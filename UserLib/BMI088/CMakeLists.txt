cmake_minimum_required(VERSION 3.30)

project(bmi088)

add_library(bmi088 INTERFACE)

target_sources(bmi088 INTERFACE
        BMI088.cpp
        BMI088_Middleware.cpp
)

target_include_directories(bmi088 INTERFACE
        ./
)