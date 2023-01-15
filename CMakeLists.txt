cmake_minimum_required(VERSION 3.9)
project(carla_bridge_app)

set(CMAKE_CXX_FLAGS "-std=c++14 -w -g")
find_package(Boost REQUIRED System)
find_package(OpenDDS REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/build/opendds_generated)

add_library(CarlaDataIDL)
OPENDDS_TARGET_SOURCES(CarlaDataIDL PUBLIC "CarlaData.idl")
target_link_libraries(CarlaDataIDL PUBLIC OpenDDS::Dcps)

set(opendds_libs
  OpenDDS::Dcps # Core OpenDDS Library
  OpenDDS::InfoRepoDiscovery OpenDDS::Tcp # For run_test.pl
  OpenDDS::Rtps OpenDDS::Rtps_Udp # For run_test.pl --rtps
  CarlaDataIDL
)

set(CARLA_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libcarla-install/lib)
set(
    CARLA_LIBS
    ${CARLA_LIB_DIR}/libcarla_client.a
    ${CARLA_LIB_DIR}/librpc.a
    ${CARLA_LIB_DIR}/libRecast.a
    ${CARLA_LIB_DIR}/libDetour.a
    ${CARLA_LIB_DIR}/libDetourCrowd.a
    ${CARLA_LIB_DIR}/libboost_filesystem.a
    ${CARLA_LIB_DIR}/libboost_system.a
    ${CARLA_LIB_DIR}/libpng16.a
    ${CARLA_LIB_DIR}/libpng.a
)
set(
    CARLA_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/libcarla-install/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libcarla-install/include/system
)

add_executable(carla_bridge_app ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/listener.cpp)
target_include_directories(carla_bridge_app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${CARLA_INCLUDE_DIR} ${BOOST_INCLUDE_DIR})
target_link_libraries(carla_bridge_app CarlaDataIDL ${CARLA_LIBS} ${BOOST_LIBRARIES})