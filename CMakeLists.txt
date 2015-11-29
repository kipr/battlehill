project(battlehill)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(Daylite REQUIRED)
find_package(Battlecreek REQUIRED)

cmake_minimum_required(VERSION 2.8)

file(GLOB BATTLEHILL_SOURCES src/*)

if(NOT WIN32)
  set(CXX_FLAGS "-std=c++11")
endif()

set(CMAKE_C_FLAGS ${C_FLAGS})
set(CMAKE_CXX_FLAGS ${CXX_FLAGS})

include_directories(${BATTLECREEK_INCLUDE_DIR} ${DAYLITE_INCLUDE_DIRS})
add_executable(battlehill ${BATTLEHILL_SOURCES})
target_link_libraries(battlehill ${DAYLITE_LIBRARIES})