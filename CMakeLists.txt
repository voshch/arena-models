cmake_minimum_required(VERSION 3.8)
project(arena_models)

if(NOT DEFINED EDITABLE)
    set(EDITABLE ON)
endif()

find_package(ament_cmake REQUIRED)

find_package(ament_cmake_venv REQUIRED)
find_package(ament_cmake_venv_pyenv REQUIRED)

file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/.python-version" PYTHON_VERSION_CONTENT)
string(STRIP "${PYTHON_VERSION_CONTENT}" PYTHON_VERSION)
pyenv_venv(PYTHON ${PYTHON_VERSION})

if(EDITABLE)
    venv_pip_install_editable("${CMAKE_CURRENT_SOURCE_DIR}/src[build]")
else()
    venv_pip_install("${CMAKE_CURRENT_SOURCE_DIR}/src[build]")
endif()

venv_install()
venv_script("arena_models" "-m arena_models")

ament_package()