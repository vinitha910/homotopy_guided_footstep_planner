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

#ifndef SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_RVIZ_VISUALIZE_H_
#define SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_RVIZ_VISUALIZE_H_

#include <footstep_planner/graphs/nav_lattice_8D.h>
#include <footstep_planner/environment/environment_interpreter.h>
#include <footstep_planner/planners/hbsp.h>
#include <footstep_planner/planners/dijkstra.h>
#include <environment_projections.pb.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <vector>
#include <string>

namespace footstep_planner {
namespace visualize {

// This function publishes a static transform from the frame id of the
// parent frame to the frame id of the child frame
void publish_static_transform(
    const std::string& from,
    const std::string& to,
    const tf::Vector3& translation,
    const tf::Quaternion& rotation);

// This function publishes a child frame
void publish_child_frame(
    const ros::NodeHandle& nh,
    const std::string& global_frame_id,
    const std::string& child_frame_id);

// This function visualizes robot model at the given pose
void visualize_robot_model(
    const ros::NodeHandle& nh,
    const std::string& global_frame_id,
    const double& x,
    const double& y,
    const double& z,
    const double& theta);

// Visualizes a cylinder for the goal region
void visualize_goal_region(
    const ros::NodeHandle& nh,
    const std::string& global_frame_id,
    const ros::Publisher& goal_pub,
    const double& x,
    const double& y,
    const double& z);

// Visualized the transformed collision spheres
void visualize_collision_model(
    const std::vector<Eigen::Vector4d>& transformed_spheres,
    const std::string& global_frame_id,
    const ros::Publisher& expansions_pub);

// This function visualizes the world mesh
void visualize_obstacles(
    const std::string& obstacle_mesh,
    const std::string& global_frame_id,
    const ros::Publisher& obstacles_vis_pub);

// This function visualizes all cells that are a valid place for the robot to
// put its feet
void visualize_stepping_cells(
    const ros::NodeHandle &nh,
    const std::string& global_frame_id,
    const std::shared_ptr<smpl::SparseDistanceMap> distance_map,
    const std::shared_ptr<environment::proto::EnvironmentProjections>
        env_projections,
    const std::shared_ptr<environment::proto::ValidSteppingCells>
        valid_stepping_cells,
    const ros::Publisher& stepping_cells_pub);

// Visualizes the platforms in the world
void visualize_platforms(
    const std::string& platform_mesh,
    const std::string& global_frame_id,
    const ros::Publisher& platforms_vis_pub);

// Visualizes a colormap of the hbsp heuristic values where green is the lowest
// cost and red is the highest cost
void visualize_hbsp_heuristic_values(
    const ros::NodeHandle& nh,
    const std::string& global_frame_id,
    const std::shared_ptr<smpl::SparseDistanceMap> distance_map,
    const std::shared_ptr<environment::EnvironmentInterpreter>
        environment_interpreter,
    const std::shared_ptr<graphs::NavLattice2D> environment,
    const std::shared_ptr<planners::HBSP> hbsp,
    const ros::Publisher& h_vals_pub);

// Visualizes a colormap of the backwards dijkstra heuristic values where green
// is the lowest cost and red is the highest cost
void visualize_dijkstra_heuristic_values(
    const ros::NodeHandle& nh,
    const std::string& global_frame_id,
    const std::shared_ptr<smpl::SparseDistanceMap> distance_map,
    const std::shared_ptr<environment::EnvironmentInterpreter>
        environment_interpreter,
    const std::shared_ptr<graphs::NavLattice2D> environment,
    const std::shared_ptr<planners::Dijkstra> dijkstra,
    const ros::Publisher& h_vals_pub);

// Visualizes the robot feet
void visualize_feet(
    const int& hidx,
    const Eigen::Vector4d left_foot_pos,
    const Eigen::Vector4d right_foot_pos,
    const std::string& global_frame_id,
    const ros::Publisher& expansions_pub);

// Visualizes the final path output by the planner
void visualize_path(
    const std::vector<int>& solution_stateIDs_V,
    const std::shared_ptr<graphs::NavLattice8D> environment,
    const std::string& global_frame_id,
    const ros::Publisher& path_pub);

}  // namespace visualize
}  // namespace footstep_planner

#endif  // SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_RVIZ_VISUALIZE_H_
