cmake_minimum_required(VERSION 3.10)
project(clipper VERSION 0.2.4)

set(CMAKE_CXX_STANDARD 14)
if(NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose build type." FORCE)
endif()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

if(NOT clipper_FIND_QUIETLY)
  message(STATUS "CLIPPER v${PROJECT_VERSION}")
endif()

###############################################################################
# Options
###############################################################################

option(CLIPPER_BUILD_BINDINGS_PYTHON "Build Python bindings" OFF)
option(CLIPPER_BUILD_BINDINGS_MATLAB "Build MATLAB bindings" OFF)
option(CLIPPER_BUILD_TESTS "Build testsuite" OFF)
option(CLIPPER_BUILD_BENCHMARKS "Build benchmarks" OFF)
option(CLIPPER_ENABLE_MKL "Use MKL with Eigen" OFF)
option(CLIPPER_ENABLE_BLAS "Use BLAS with Eigen" OFF) # apt install libopenblas-dev
option(CLIPPER_ENABLE_MAXCLIQUE "Use PMC lib for solveAsMaximumClique" ON)
option(CLIPPER_ENABLE_SCS_SDR "Use SCS lib for solveAsMSRCSDR" ON)

if(CLIPPER_ENABLE_MKL AND CLIPPER_ENABLE_BLAS)
  message(FATAL_ERROR "Cannot enable both MKL and BLAS (prefer MKL)")
endif()

###############################################################################
# Dependencies
###############################################################################

##########Triangulaton library##############
# catkin dependency removed for ROS2 Jazzy port — clipper is built as a plain CMake subdirectory
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
INCLUDE_DIRECTORIES(/usr/local/include/libqhullcpp)
INCLUDE_DIRECTORIES(/usr/local/include/libqhull_r)
include_directories(${OpenCV_INCLUDE_DIRS})
SET(qhullLibs qhullcpp qhull_r)
LINK_DIRECTORIES(/usr/local/lib)

##########Triangulaton library##############


include(FetchContent)

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen Version: ${EIGEN3_VERSION_STRING} ${EIGEN3_VERSION}")

find_package(OpenMP)

# Give preference to MKL
if(CLIPPER_ENABLE_MKL)
    find_package(MKL)
    if(MKL_FOUND)
        message(STATUS "MKL found at: ${MKL_LIBRARIES}")
    else()
        message(STATUS "MKL not found.")
    endif()
endif()

if(CLIPPER_ENABLE_BLAS)
    # OpenBLAS is generally better than ATLAS
    set(BLA_VENDOR OpenBLAS)
    find_package(BLAS)
    if(BLAS_FOUND)
        message(STATUS "BLAS found at: ${BLAS_LIBRARIES}")
    else()
        message(STATUS "BLAS not found.")
    endif()
endif()

if(CLIPPER_BUILD_BINDINGS_PYTHON)
  FetchContent_Declare(pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG        v2.9.2
  )
  FetchContent_MakeAvailable(pybind11)
endif()

if(CLIPPER_ENABLE_MAXCLIQUE)
    FetchContent_Declare(pmc
        GIT_REPOSITORY  https://github.com/jingnanshi/pmc
        GIT_TAG         16af470353a784352760ec2deea25fa7ed602450)
    FetchContent_MakeAvailable(pmc)
endif()

if(CLIPPER_ENABLE_SCS_SDR)
    include(FetchContent)
    FetchContent_Declare(scs
        GIT_REPOSITORY  https://github.com/cvxgrp/scs
        GIT_TAG         3.2.3)
    FetchContent_MakeAvailable(scs)
endif()

###############################################################################
# Targets
###############################################################################

add_library(clipper SHARED)
target_sources(clipper PRIVATE
  src/clipper.cpp
  src/utils.cpp
  src/dsd.cpp
  src/sdp.cpp
  src/maxclique.cpp
  src/invariants/euclidean_distance.cpp
  src/invariants/pointnormal_distance.cpp
  src/semantic_clipper.cpp)
target_link_libraries(clipper PUBLIC Eigen3::Eigen)
target_link_libraries(clipper PRIVATE OpenMP::OpenMP_CXX)
target_include_directories(clipper PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
# Provide no-op shims for ROS1 logging macros that still appear in this
# vendored third-party source so the file compiles without roscpp.
target_compile_definitions(clipper PRIVATE
  "ROS_INFO_STREAM(x)=do{}while(0)"
  "ROS_WARN_STREAM(x)=do{}while(0)"
  "ROS_ERROR_STREAM(x)=do{}while(0)"
  "ROS_DEBUG_STREAM(x)=do{}while(0)")
target_compile_options(clipper PUBLIC -march=native -mavx -mfma)
set_target_properties(clipper PROPERTIES
  VERSION ${PROJECT_VERSION})
target_compile_definitions(clipper PUBLIC CLIPPER_VERSION="${PROJECT_VERSION}")

if(MKL_FOUND)
    target_include_directories(clipper PRIVATE ${MKL_INCLUDE_DIR})
    target_compile_definitions(clipper PRIVATE -DEIGEN_USE_MKL_ALL)
    target_link_libraries(clipper PRIVATE ${MKL_LIBRARIES})
elseif(BLAS_FOUND)
    target_compile_definitions(clipper PRIVATE -DEIGEN_USE_BLAS=1)
    target_link_libraries(clipper PRIVATE BLAS::BLAS)
endif()

if(CLIPPER_ENABLE_MAXCLIQUE)
    target_compile_definitions(clipper PUBLIC CLIPPER_HAS_PMC)
    target_link_libraries(clipper PRIVATE pmc)
endif()

if(CLIPPER_ENABLE_SCS_SDR)
    target_compile_definitions(clipper PUBLIC CLIPPER_HAS_SCS)
    target_link_libraries(clipper PRIVATE scs::scsdir)
endif()

##########Triangulaton target##############
add_library(triangulation SHARED)
target_sources(triangulation PRIVATE
  src/triangulation/descriptor.cpp
  src/triangulation/observation.cpp)
target_include_directories(triangulation PRIVATE include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(triangulation ${OpenCV_LIBS} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} ${qhullLibs})

add_executable(tripper examples/c++/example.cpp)
target_link_libraries(tripper triangulation clipper ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES} ${qhullLibs})
target_include_directories(tripper PRIVATE include/clipper include/triangulation ${EIGEN3_INCLUDE_DIR})

##########Semantic Clipper target##############
add_library(semantic_clipper SHARED)
target_sources(semantic_clipper PRIVATE
  src/semantic_clipper.cpp)
target_link_libraries(semantic_clipper PRIVATE clipper triangulation ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})
target_include_directories(semantic_clipper PRIVATE include ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
# Provide no-op shims for ROS1 logging macros that still appear in this
# vendored third-party source so the file compiles without roscpp.
target_compile_definitions(semantic_clipper PRIVATE
  "ROS_INFO_STREAM(x)=do{}while(0)"
  "ROS_WARN_STREAM(x)=do{}while(0)"
  "ROS_ERROR_STREAM(x)=do{}while(0)"
  "ROS_DEBUG_STREAM(x)=do{}while(0)")

#########Test semantic clipper target##############
add_executable(test_semantic_clipper test/test_semantic_clipper.cpp)
target_link_libraries(test_semantic_clipper semantic_clipper ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})
target_include_directories(test_semantic_clipper PRIVATE include ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

###############################################################################
# Extras
###############################################################################

if(CLIPPER_BUILD_BINDINGS_PYTHON)
    message(STATUS "Building Python bindings.")
    add_subdirectory(bindings/python)
endif()

if(CLIPPER_BUILD_BINDINGS_MATLAB)
    message(STATUS "Attempting to build MATLAB bindings.")
    add_subdirectory(bindings/matlab)
endif()

if(CLIPPER_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

if(CLIPPER_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()

###############################################################################
# Install
###############################################################################

include(GNUInstallDirs)
message(STATUS "CMAKE_INSTALL_LIBDIR: ${CMAKE_INSTALL_LIBDIR}")
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/clipper)

install(TARGETS clipper
    EXPORT clipper-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT clipper-targets
  FILE clipper-targets.cmake
  DESTINATION ${INSTALL_CONFIGDIR})

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/clipper-config-version.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/clipper-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/clipper-config.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR})

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/clipper-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/clipper-config-version.cmake
  DESTINATION ${INSTALL_CONFIGDIR})
