cmake_minimum_required(VERSION 3.8)
project(arena_models)

find_package(ament_cmake REQUIRED)

find_package(ament_cmake_venv REQUIRED)
find_package(ament_cmake_venv_pyenv REQUIRED)

file(GLOB_RECURSE SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*")

# clean rebuilds

execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/src ${venv_build_dir}
)
  
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/.python-version" PYTHON_VERSION_CONTENT)
string(STRIP "${PYTHON_VERSION_CONTENT}" PYTHON_VERSION)
pyenv_venv(PYTHON ${PYTHON_VERSION})
venv_pip_install(${venv_build_dir})
venv_python(-m spacy download en_core_web_md)
venv_install()
venv_script("arena_models" "-m arena_models")

ament_package()