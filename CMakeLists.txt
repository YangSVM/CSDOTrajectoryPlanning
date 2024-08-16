cmake_minimum_required(VERSION 3.5)

project(PbsTrajectoryPlanning)
# list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_finders")

find_package(Boost 1.65   REQUIRED COMPONENTS program_options)
find_package(Eigen3  3.4   REQUIRED)
find_package(PkgConfig)
find_package(osqp REQUIRED)

pkg_check_modules(YamlCpp yaml-cpp)

## OPEN MOTION PLANNING LIBRARY
find_package(ompl REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)


# add mkl for eigen to calculate faster.
# enable Intel-MKL (only for Intel compilers)
# NB requires correct specification of BLA_VENDOR, e.g.
#   export BLA_VENDOR=Intel10_64lp_seq
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
  find_package(BLAS)
  include_directories(${BLAS_INCLUDE_DIRS})
  set(PROJECT_LIBS ${BLAS_LIBRARIES})
  add_definitions(-DEIGEN_USE_MKL_ALL)
endif()


# Enable C++14 and warnings
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if (${CMAKE_BUILD_TYPE} MATCHES "Release")
  message("Release mode")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")
else(${CMAKE_BUILD_TYPE} MATCHES "Release")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif (${CMAKE_BUILD_TYPE} MATCHES "Release") 

# Creates compile database used by clang-tidy.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
  ./
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

file(GLOB SOURCES  "hybrid_a_star/*.cc" "util/*.cc")
file(GLOB PBS_SOURCES  "pbs/*.cc")

# Examples
add_executable(sp 
  ${SOURCES}
  sequantial_planner.cc
)

target_link_libraries(sp ${Boost_LIBRARIES}   yaml-cpp   ${OMPL_LIBRARIES}
)


add_executable(pbs
  ${SOURCES}
  ${PBS_SOURCES}
  pbs_search.cc
)

target_link_libraries(pbs ${Boost_LIBRARIES}   yaml-cpp   ${OMPL_LIBRARIES}
)

file(GLOB QP_SOURCES  "qp/*.cc")
# CL_ECBS-nlp
add_executable(csdo
  ${SOURCES}  
  ${PBS_SOURCES}
  ${QP_SOURCES}
  csdo.cc
)
target_link_libraries(csdo
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
  Eigen3::Eigen
  yaml-cpp
  osqp::osqp
)
