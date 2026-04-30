cmake_minimum_required(VERSION 3.30)

project(storage_embedded)

add_library(storage_embedded INTERFACE)

target_sources(storage_embedded INTERFACE
        Storage_EmbeddedFlash.cpp
)

target_include_directories(storage_embedded INTERFACE
        ./
)