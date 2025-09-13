cmake_minimum_required(VERSION 3.8)
project(arena_models)

find_package(ament_cmake REQUIRED)

find_package(ament_cmake_venv REQUIRED)

execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/src ${venv_build_dir}
)
execute_process(
  COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/scripts/build.sh ${venv_build_dir} ${venv_dir}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  RESULT_VARIABLE build_sh_result
)
if(NOT build_sh_result EQUAL 0)
  message(FATAL_ERROR "scripts/build.sh failed with exit code ${build_sh_result}")
endif()

venv_install()
venv_script("arena_models" "-m arena_models")

ament_package()