cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(avci_net_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Gazebo Harmonic (gz-sim 8)
find_package(gz-cmake3 REQUIRED)
gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
gz_find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
gz_find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})
gz_find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

foreach(plugin NetCapturePlugin NetLauncherPlugin)
  add_library(${plugin} SHARED ${plugin}.cc)
  target_include_directories(${plugin} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(${plugin} PRIVATE
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    gz-plugin${GZ_PLUGIN_VER}::register
    gz-transport${GZ_TRANSPORT_VER}::core
    gz-msgs${GZ_MSGS_VER}::core
    gz-common${GZ_COMMON_VER}::profiler
  )
endforeach()
