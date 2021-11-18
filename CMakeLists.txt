cmake_minimum_required(VERSION 3.15)
project(sm64_bully_movement_v2
  LANGUAGES CXX
  DESCRIPTION "Bruteforces bully speed/angle combinations for triple-bully-collision displacement. Now with less precision!"
)

# Set default build type to Release.
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

add_executable(bruteforce
  main.cpp
  vmath.cpp
  vmath.hpp
  WallsFloors.cpp
  WallsFloors.hpp
  Trig.cpp
  Trig.hpp
  SurfaceUtils.cpp
  SurfaceUtils.hpp
  Surface.cpp
  Surface.hpp
  PathSearch.cpp
  PathSearch.hpp
  Constants.hpp
  BullyPath.cpp
  BullyPath.hpp
)

# Equivalent to specifying a bunch of compiler flags.
# ===================================================

set_target_properties(bruteforce PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED true
)

# LTO
include(CheckIPOSupported)
check_ipo_supported(RESULT lto_supported OUTPUT lto_error LANGUAGES CXX)
if(lto_supported)
  set_target_properties(bruteforce PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION true
  )
endif()


# OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  target_link_libraries(bruteforce PRIVATE OpenMP::OpenMP_CXX)
endif()

# -march=native with MSVC emulation
include("cmake/march_native.cmake")
set_march_native(bruteforce)

# -O3 is already handled by using release mode.