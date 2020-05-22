cmake_minimum_required(VERSION 2.8.9)
project (ompl_tutorials)

add_compile_options(-std=c++14)

# use specific installation of ompl with absolute path
set(OMPL_INCLUDE_DIR /home/jeroen/Applications/ompl/usr/local/include/ompl-1.5)
set(OMPL_LIB_DIR /home/jeroen/Applications/ompl/usr/local/lib)

include_directories(
  /usr/include/eigen3
  include
  ${OMPL_INCLUDE_DIR}
)

link_directories(
  ${OMPL_LIB_DIR}
)

add_executable(
  working
  working.cpp
)

target_link_libraries(
  working
  ompl
)