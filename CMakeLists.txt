cmake_minimum_required(VERSION 3.8)
project(arena_models)

find_package(ament_cmake REQUIRED)

find_package(ament_cmake_venv REQUIRED)

file(GLOB_RECURSE SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*")

add_custom_target(copy_src_dir
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/src ${venv_build_dir}
  DEPENDS ${SRC_FILES}
  COMMENT "Copying source directory to venv build directory"
)

add_custom_target(run_build_script ALL
  COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/scripts/build.sh ${venv_build_dir} ${venv_dir}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS copy_src_dir ${CMAKE_CURRENT_SOURCE_DIR}/scripts/build.sh
  BYPRODUCTS ${venv_dir}
  COMMENT "Running build.sh script"
)

venv_install()
venv_script("arena_models" "-m arena_models")

ament_package()