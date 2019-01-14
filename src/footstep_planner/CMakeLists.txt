cmake_minimum_required(VERSION 3.0)
project(footstep_planner)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        eigen_conversions
        geometry_msgs
        geometric_shapes
        sbpl_collision_checking
        nav_msgs
        pcl_ros
        roscpp
        tf)

find_package(PythonInterp)
find_package(Eigen REQUIRED)
find_package(sbpl REQUIRED)
find_package(smpl REQUIRED)

set(sbpl_INCLUDE_DIRS ${SBPL_INCLUDE_DIRS})
set(sbpl_LIBRARIES ${SBPL_LIBRARIES})

INCLUDE(FindProtobuf)
FIND_PACKAGE(Protobuf REQUIRED)

catkin_package()

PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER
    src/proto/environment_projections/environment_projections.proto
    src/proto/homotopy_information/homotopy_information.proto
    src/proto/robot_parameters/robot_parameters.proto)

PROTOBUF_GENERATE_PYTHON(PROTO_PY
    src/proto/environment_projections/environment_projections.proto
    src/proto/homotopy_information/homotopy_information.proto
    src/proto/robot_parameters/robot_parameters.proto)

add_custom_target(python_protobufs ALL DEPENDS ${PROTO_PY})

add_custom_command(
    TARGET python_protobufs POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        ${PROTO_PY}
        ${CMAKE_CURRENT_SOURCE_DIR}/python/proto)

include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${sbpl_INCLUDE_DIRS})
include_directories(include)

add_executable(footstep_planner
    src/test/footstep_planner_test.cpp
    src/environment/environment_interpreter.cpp
    src/graphs/homotopy_information.cpp
    src/graphs/nav_lattice_2D.cpp
    src/graphs/nav_lattice_8D.cpp
    src/planners/hbsp.cpp
    src/planners/dijkstra.cpp
    src/planners/mha.cpp
    src/heuristics/homotopy_based_heuristic.cpp
    src/heuristics/backward_dijkstra_heuristic.cpp
    src/utils/state_conversions.cpp
    src/proto/homotopy_information/pack_into_proto.cpp
    src/proto/homotopy_information/unpack_from_proto.cpp
    src/proto/environment_projections/pack_into_proto.cpp
    src/proto/environment_projections/unpack_from_proto.cpp
    src/proto/robot_parameters/unpack_from_proto.cpp
    rviz/visualize.cpp
    ${PROTO_HEADER}
    ${PROTO_SRC})

add_executable(project_environment_test
    src/test/project_environment_test.cpp
    src/environment/environment_interpreter.cpp
    src/graphs/homotopy_information.cpp
    src/graphs/nav_lattice_2D.cpp
    src/graphs/nav_lattice_8D.cpp
    src/utils/state_conversions.cpp
    src/proto/homotopy_information/pack_into_proto.cpp
    src/proto/homotopy_information/unpack_from_proto.cpp
    src/proto/environment_projections/pack_into_proto.cpp
    src/proto/environment_projections/unpack_from_proto.cpp
    src/proto/robot_parameters/unpack_from_proto.cpp
    rviz/visualize.cpp
    ${PROTO_HEADER}
    ${PROTO_SRC})

target_link_libraries(footstep_planner ${catkin_LIBRARIES} ${sbpl_collision_checking_LIBRARIES} ${sbpl_LIBRARIES} ${PROTOBUF_LIBRARIES})
target_link_libraries(project_environment_test ${catkin_LIBRARIES} ${sbpl_collision_checking_LIBRARIES} ${sbpl_LIBRARIES} ${PROTOBUF_LIBRARIES})

install(TARGETS footstep_planner RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS project_environment_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scenarios DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
