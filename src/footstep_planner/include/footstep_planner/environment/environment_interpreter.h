////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019, Vinitha Ranganeni & Sahit Chintalapudi
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

#ifndef SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_ENVIRONMENT_ENVIRONMENT_INTERPRETER_H_
#define SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_ENVIRONMENT_ENVIRONMENT_INTERPRETER_H_

#include <footstep_planner/utils/helpers.h>
#include <environment_projections.pb.h>
#include <robot_parameters.pb.h>
#include <smpl/distance_map/sparse_distance_map.h>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>
#include <map>

typedef std::shared_ptr<smpl::SparseDistanceMap> SparseDistanceMapPtr;

namespace footstep_planner {
namespace environment {

using projection_proto = proto::EnvironmentProjection;
using projections_proto = proto::EnvironmentProjections;
using cell_type_proto = proto::EnvironmentProjection_CellType;

// This class interprets the world into 3D and 2D representations that can
// be used for generating 2D heuristics and planning efficiently in 3D.
class EnvironmentInterpreter {
 public:
    // Initializes the discretized world (mesh) at the specified resolution
    // in map_params
    //
    // platforms The path to the platforms mesh
    // obstacles The path to the obstacles mesh
    // surfaces A vector of paths to the surface meshes
    // stepping_surfaces A vector of paths to the stepping_surface meshes
    // robot_details_proto The robot parameters protobuf
    EnvironmentInterpreter(
        const std::string& platforms,
        const std::string& obstacles,
        const std::vector<std::string>& surfaces,
        const std::vector<std::string>& stepping_surfaces,
        const std::map<std::string, double>& map_params,
        const footstep_planner::proto::RobotParameters& robot_details_proto);

    // The function pessimistically projects the the 3D environment onto the
    // xy-plane (i.e. a cell (x, y, z) is projected as obstacle cell (x, y)
    // in 2D if there is at least one z-value for which (x, y, z) is
    // occupied)
    void project_environment();

    // Projects the environment and finds the stepping cells, packs them into
    // protobufs and writes them out to their respective files
    bool project_and_save_environment(
        const char* projection_out_file,
        const char* stepping_cells_out_file);

    // Reads and unpacks the projection and stepping cells protobufs
    bool read_projection(
        const char* projection_in_file,
        const char* stepping_cells_in_file);

    std::shared_ptr<projections_proto> get_environment_projections()
        const { return env_projections_ptr_; }

    std::shared_ptr<proto::ValidSteppingCells> get_valid_stepping_cells()
        const { return stepping_cells_ptr_; }

    SparseDistanceMapPtr get_distance_map() const { return distance_map_; }

 private:
    // This function reads a mesh and creates a vector of voxels for each
    // surface
    //
    // mesh The path the mesh file
    // voxels The vector of voxels to be filled in
    // vertices The vector of vertices to be filled in
    void voxelize_world(
        const std::string& mesh,
        std::vector<Eigen::Vector3d>* voxels,
        std::vector<Eigen::Vector3d>* vertices);

    // Creates a distance map pointer using map_params
    SparseDistanceMapPtr create_distance_map();

    // Finds the edges of a surface
    void find_surface_edges();

    // The function pessimistically projects obstacles in the projection
    void project_obstacles(
        const std::vector<std::vector<int>>& workspace_2d_to_surface_3d,
        const std::vector<int>& surface_3d_to_stepping_cells,
        std::vector<std::vector<cell_type_proto>>* projections);

    // Inflates obstables using the robot in-radius
    void inflate_obstacles(
        std::vector<std::vector<cell_type_proto>>* projections);

    // Computes a 2D distance transform, i.e. for each cells, it records the
    // distance to the nearest obstacle cell
    void compute_distance_transform(
        const std::vector<std::vector<cell_type_proto>>& projections,
        std::vector<int>* distance_transform_2d);

    // Projects the surface boundaries/edges
    void project_surface_boundaries(
        const int& workspace_idx,
        const std::vector<utils::Coordinates3DPairs>& edge_points,
        std::vector<cell_type_proto>* projection);

    // Creates a mapping from (x, y, surface) to a valid stepping point
    void create_valid_stepping_cells_mapping(
        std::vector<bool>* stepping_cells);

    // Creates mapping from (x, y, platform_z) to (x, y, workspace_idx)
    void create_workspace_3d_to_2d_mapping(
        std::vector<int>* workspace_3d_to_2d,
        std::vector<std::vector<int>>* workspace_2d_to_surface_3d);

    // Creates a mapping from (x, y, surface_z) to the nearest valid stepping
    // cell on the platform
    void create_surface_3d_to_stepping_cells_mapping(
        const std::vector<std::vector<int>>& workspace_2d_to_surface_3d,
        const std::vector<bool>& stepping_cells,
        std::vector<int>* surface_3d_to_stepping_cells);

    // Creates a mapping from (x, y, surface_index) to (x, y, surface_z)
    void create_surface_2d_and_3d_mappings(
        const int& workspace_idx,
        std::vector<std::vector<cell_type_proto>>* projections,
        std::vector<std::vector<int>>* workspace_2d_to_surface_3d,
        std::vector<std::pair<int, int>>* surface_3d_to_indices);

    // Given two points, finds an equation of a line
    void find_line_eq(
        const utils::Coordinates3D& coor1,
        const utils::Coordinates3D& coor2,
        double* m,
        double* b);

    // Discretized robot height
    int robot_height_;

    // The continuous robot in_radius (meters)
    double robot_in_radius_m_;

    // The parameters of the map/projections
    std::map<std::string, double> map_params_;

    // The maximum collision sphere radius
    double max_collision_sphere_radius_;

    // A vector of voxels of the obstacles
    std::vector<Eigen::Vector3d> obstacle_voxels_;

    // A vector of voxels for each surface and each stepping surface
    std::vector<std::vector<Eigen::Vector3d>> stepping_surface_voxels_;

    // A vectors of corners for each surface
    std::vector<std::vector<Eigen::Vector3d>> surface_corners_;

    // A vector of two points that bound each edge of the surface
    std::vector<std::vector<utils::Coordinates3DPairs>> surface_edge_points_;

    // A protobuf containing the projected environment
    proto::EnvironmentProjections env_projections_;
    std::shared_ptr<proto::EnvironmentProjections> env_projections_ptr_;

    // A protobuf containing a list of cells the robot can step on
    proto::ValidSteppingCells stepping_cells_;
    std::shared_ptr<proto::ValidSteppingCells> stepping_cells_ptr_;

    // A vector of distance maps for each surface
    std::vector<SparseDistanceMapPtr> surface_distance_maps_;

    // Distance map for the entire world
    SparseDistanceMapPtr distance_map_;
};

}  // namespace environment
}  // namespace footstep_planner

#endif  // SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_ENVIRONMENT_ENVIRONMENT_INTERPRETER_H_
