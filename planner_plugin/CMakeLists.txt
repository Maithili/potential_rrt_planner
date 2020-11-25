cmake_minimum_required (VERSION 2.6.0)
project(planner_plugin)
find_package(OpenRAVE 0.9 REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

set(CMAKE_BUILD_TYPE Debug)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_library(planner_plugin SHARED 
  planner_plugin_interface.cpp
  libcd/chomp.c
  libcd/grid.c
  libcd/grid_flood.c
  libcd/kin.c
  libcd/mat.c
  libcd/os.c
  libcd/spatial.c
  libcd/util.c
  libcd/util_shparse.c
)
set_target_properties(planner_plugin PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(planner_plugin ${OpenRAVE_LIBRARIES})
install(TARGETS planner_plugin DESTINATION .)
