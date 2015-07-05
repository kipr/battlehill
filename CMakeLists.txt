project(battlehill)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake") 
find_package(Daylite REQUIRED)

find_path(BATTLECREEK_INCLUDE_DIR battlecreek/battlecreek.hpp)


cmake_minimum_required(VERSION 2.8)

file(GLOB BATTLEHILL_SOURCES src/*)

add_definitions(-std=c++11)
include_directories(${BATTLECREEK_INCLUDE_DIR} ${DAYLITE_INCLUDE_DIRS})
add_executable(battlehill ${BATTLEHILL_SOURCES})
target_link_libraries(battlehill ${DAYLITE_LIBRARIES})