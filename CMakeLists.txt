cmake_minimum_required(VERSION 2.8.12)

set(CMAKE_CXX_STANDARD 11)

project(xythetac)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})


add_executable(xythetac src/xythetaARA.cpp src/environment_navxythetaC.cpp src/dubins.cpp)

target_link_libraries(xythetac "/usr/local/lib/libsbpl.so")