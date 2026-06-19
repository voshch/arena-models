cmake_minimum_required(VERSION 3.8)
project(arena_models)

find_package(ament_cmake REQUIRED)

find_package(ament_cmake_venv REQUIRED)
find_package(ament_cmake_venv_uv REQUIRED)

uv_venv(DIRECTORY src)

venv_script("arena_models" "-m arena_models")

ament_package()
