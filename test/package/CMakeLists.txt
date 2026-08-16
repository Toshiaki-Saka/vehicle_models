# Smoke test for the installed package: find_package + link + run.
cmake_minimum_required(VERSION 3.16)
project(vehicle_models_package_test LANGUAGES CXX)

find_package(vehicle_models 0.1 REQUIRED)

add_executable(package_test main.cpp)
target_link_libraries(package_test PRIVATE vehicle_models::vehicle_models)
