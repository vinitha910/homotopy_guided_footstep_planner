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

#include <footstep_planner/rviz/visualize.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <leatherman/utils.h>

namespace footstep_planner {
namespace visualize {

void publish_static_transform(
    const std::string& from,
    const std::string& to,
    const tf::Vector3& translation,
    const tf::Quaternion& rotation) {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = from;
    static_transformStamped.child_frame_id = to;
    static_transformStamped.transform.translation.x = translation.getX();
    static_transformStamped.transform.translation.y = translation.getY();
    static_transformStamped.transform.translation.z = translation.getZ();
    static_transformStamped.transform.rotation.x = rotation.x();
    static_transformStamped.transform.rotation.y = rotation.y();
    static_transformStamped.transform.rotation.z = rotation.z();
    static_transformStamped.transform.rotation.w = rotation.w();
    static_broadcaster.sendTransform(static_transformStamped);
}

void publish_child_frame(
    const ros::NodeHandle& nh,
    const std::string& global_frame_id,
    const std::string& child_frame_id) {
    tf::Vector3 trans(0, 0, 0);
    tf::Quaternion rot;
    rot.setRPY(0, 0, 0);

    publish_static_transform(global_frame_id, child_frame_id, trans, rot);
}

void visualize_collision_model(
    const std::vector<Eigen::Vector4d>& transformed_spheres,
    const std::string& global_frame_id,
    const ros::Publisher& expansions_pub) {
    visualization_msgs::MarkerArray markers;

    int sphere_idx = 0;
    for (const auto sphere : transformed_spheres) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = global_frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "collision_spheres_" + std::to_string(sphere_idx);
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = sphere.x();
        marker.pose.position.y = sphere.y();
        marker.pose.position.z = sphere.z();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = sphere.w();
        marker.scale.y = sphere.w();
        marker.scale.z = sphere.w();
        marker.color.r = 35.0/255.0;
        marker.color.g = 148.0/255.0;
        marker.color.b = 140.0/255.0;
        marker.color.a = 0.6;
        markers.markers.push_back(marker);
        sphere_idx++;
    }
    expansions_pub.publish(markers);
}

void visualize_goal_region(
    const ros::NodeHandle& nh,
    const std::string& global_frame_id,
    const ros::Publisher& goal_pub,
    const double& x,
    const double& y,
    const double& z) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = global_frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "goal_region";
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 2.0;
    marker.color.r = 0.0/255.0;
    marker.color.g = 138.0/255.0;
    marker.color.b = 40.0/255.0;
    marker.color.a = 0.7;

    goal_pub.publish(marker);
}

void visualize_robot_model(
    const ros::NodeHandle& nh,
    const std::string& global_frame_id,
    const double& x,
    const double& y,
    const double& z,
    const double& theta) {
    tf::Vector3 trans_urdf(x, y, z);

    tf::Quaternion rot_urdf;
    rot_urdf.setRPY(0, 0, theta);

    std::vector<std::string> frames;
    nh.getParam("frames", frames);
    for (const auto& frame : frames) {
        publish_static_transform(global_frame_id, frame, trans_urdf, rot_urdf);
    }
}

void visualize_obstacles(
    const std::string& obstacle_mesh,
    const std::string& global_frame_id,
    const ros::Publisher& obstacles_vis_pub) {
    visualization_msgs::Marker obstacle_mesh_marker;
    obstacle_mesh_marker.header.stamp = ros::Time::now();
    obstacle_mesh_marker.header.frame_id = global_frame_id;
    obstacle_mesh_marker.ns = "obstacles";
    obstacle_mesh_marker.id = 0;
    obstacle_mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    obstacle_mesh_marker.action = visualization_msgs::Marker::ADD;
    obstacle_mesh_marker.scale.x = 1.0;
    obstacle_mesh_marker.scale.y = 1.0;
    obstacle_mesh_marker.scale.z = 1.0;
    obstacle_mesh_marker.lifetime = ros::Duration(0.0);
    obstacle_mesh_marker.frame_locked = false;
    obstacle_mesh_marker.pose.orientation.w = 1.0;
    obstacle_mesh_marker.color.r = 74.0/255.0;
    obstacle_mesh_marker.color.g = 76.0/255.0;
    obstacle_mesh_marker.color.b = 76.0/255.0;
    obstacle_mesh_marker.color.a = 1.0f;
    obstacle_mesh_marker.mesh_resource = obstacle_mesh;

    obstacles_vis_pub.publish(obstacle_mesh_marker);
}

void visualize_stepping_cells(
    const ros::NodeHandle &nh,
    const std::string& global_frame_id,
    const std::shared_ptr<smpl::SparseDistanceMap> distance_map,
    const std::shared_ptr<environment::proto::EnvironmentProjections>
        env_projections,
    const std::shared_ptr<environment::proto::ValidSteppingCells>
        valid_stepping_cells,
    const ros::Publisher& stepping_cells_pub) {
    std::map<std::string, double> params;
    nh.getParam("map_params", params);
    visualization_msgs::MarkerArray markers;
    const double m_scale = params["cell_size"];
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = global_frame_id;
    marker.ns = "PLATFORM";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = marker.pose.position.y =
        marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = m_scale;
    marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0f;

    const auto rows = env_projections->rows();
    const auto cols = env_projections->cols();
    for (int z = 0; z < params["size_z"] / params["cell_size"] - 1; z++) {
        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
                const int idx = (z * rows * cols) + (y * cols) + x;
                if (valid_stepping_cells->stepping_cell(idx)) {
                    double wx, wy, wz;
                    distance_map->gridToWorld(x, y, z, wx, wy, wz);
                    geometry_msgs::Point p;
                    p.x = wx; p.y = wy; p.z = wz;
                    marker.points.push_back(p);
                }
            }
        }
    }

    ROS_INFO_NAMED("Visualize Platforms",
        "Visualizing %zu platform points",
        marker.points.size());
    markers.markers.push_back(marker);
    stepping_cells_pub.publish(markers);
}

void visualize_platforms(
    const std::string& platform_mesh,
    const std::string& global_frame_id,
    const ros::Publisher& platforms_vis_pub) {
    visualization_msgs::Marker platform_mesh_marker;
    platform_mesh_marker.header.stamp = ros::Time::now();
    platform_mesh_marker.header.frame_id = global_frame_id;
    platform_mesh_marker.ns = "platforms";
    platform_mesh_marker.id = 0;
    platform_mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    platform_mesh_marker.action = visualization_msgs::Marker::ADD;
    platform_mesh_marker.scale.x = 1.0;
    platform_mesh_marker.scale.y = 1.0;
    platform_mesh_marker.scale.z = 1.0;
    platform_mesh_marker.lifetime = ros::Duration(0.0);
    platform_mesh_marker.frame_locked = false;
    platform_mesh_marker.pose.orientation.w = 1.0;
    platform_mesh_marker.color.r = 129.0/255.0;
    platform_mesh_marker.color.g = 150.0/255.0;
    platform_mesh_marker.color.b = 149.0/255.0;
    platform_mesh_marker.color.a = 1.0f;
    platform_mesh_marker.mesh_resource = platform_mesh;

    platforms_vis_pub.publish(platform_mesh_marker);
}

void visualize_dijkstra_heuristic_values(
    const ros::NodeHandle& nh,
    const std::string& global_frame_id,
    const std::shared_ptr<smpl::SparseDistanceMap> distance_map,
    const std::shared_ptr<environment::EnvironmentInterpreter>
        environment_interpreter,
    const std::shared_ptr<graphs::NavLattice2D> environment,
    const std::shared_ptr<planners::Dijkstra> dijkstra,
    const ros::Publisher& h_vals_pub) {
    std::map<std::string, double> params;
    nh.getParam("map_params", params);

    visualization_msgs::MarkerArray markers;
    const double m_scale = params["cell_size"];
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = global_frame_id;
    marker.ns = "HVALS";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = marker.pose.position.y =
        marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = m_scale;
    marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0f;

    const double max_cost = dijkstra->get_dijkstra_max_cost();

    // pre-compute colors
    std::vector<std_msgs::ColorRGBA> hue_color_map(360);
    leatherman::msgHSVToRGB(0.0, 1.0, 1.0, hue_color_map[0]);
    for (int i = 120; i < 360; ++i) {
        leatherman::msgHSVToRGB(
            static_cast<double>(i), 1.0, 1.0, hue_color_map[i]);
    }
    const double max_cost_inv = 1.0 / max_cost;
    const planners::CostMap cost_map = dijkstra->get_dijkstra_cost_map();

    const auto env_projections =
        environment_interpreter->get_environment_projections();

    const int rows = distance_map->numCellsY();
    const int cols = distance_map->numCellsX();

    for (int i = 0; i < env_projections->projections_size(); ++i) {
        const environment::proto::EnvironmentProjection projection =
            env_projections->projections(i);
        for (int x = 0; x < cols; ++x) {
            for (int y = 0; y < rows; ++y) {
                int id = environment->get_state_id(x, y, i);
                if (cost_map.find(id) == cost_map.end()) {
                    continue;
                }
                double cost = cost_map.at(id);

                const int surface_2d_idx = (y * cols) + x;
                const int surface_z =
                    projection.workspace_2d_to_surface_3d(surface_2d_idx);

                const int surface_3d_idx =
                    surface_z * rows * cols + surface_2d_idx;
                const int stepping_z =
                    env_projections->surface_3d_to_stepping_cells(
                        surface_3d_idx);

                double wx, wy, wz;
                distance_map->gridToWorld(x, y, stepping_z, wx, wy, wz);
                geometry_msgs::Point p;
                p.x = wx; p.y = wy; p.z = wz;

                marker.points.push_back(p);

                const double alpha = cost * max_cost_inv;
                int hue = static_cast<int>((120.0 + alpha * 240.0)) % 360;
                const std_msgs::ColorRGBA &col = hue_color_map[hue];
                marker.colors.push_back(col);
            }
        }
    }
    ROS_INFO_NAMED(
        "DijkstraBasedHeuristic",
        "Visualizing %zu dijkstra_based_heuristic points",
        marker.points.size());
    markers.markers.push_back(marker);

    h_vals_pub.publish(markers);
}

void visualize_feet(
    const int& hidx,
    const Eigen::Vector4d left_foot_pos,
    const Eigen::Vector4d right_foot_pos,
    const std::string& global_frame_id,
    const ros::Publisher& expansions_pub) {
    tf::Quaternion left_foot_rot;
    left_foot_rot.setRPY(0, 0, left_foot_pos.w());
    tf::Quaternion right_foot_rot;
    right_foot_rot.setRPY(0, 0, right_foot_pos.w());

    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker left_foot_marker;
    left_foot_marker.header.frame_id = global_frame_id;
    left_foot_marker.header.stamp = ros::Time::now();
    left_foot_marker.ns = "left_foot_" + std::to_string(hidx);
    left_foot_marker.id = 1;
    left_foot_marker.type = visualization_msgs::Marker::CUBE;
    left_foot_marker.action = visualization_msgs::Marker::ADD;
    left_foot_marker.pose.position.x = left_foot_pos.x();
    left_foot_marker.pose.position.y = left_foot_pos.y();
    left_foot_marker.pose.position.z = left_foot_pos.z() + 0.05;
    left_foot_marker.pose.orientation.x = left_foot_rot.x();
    left_foot_marker.pose.orientation.y = left_foot_rot.y();
    left_foot_marker.pose.orientation.z = left_foot_rot.z();
    left_foot_marker.pose.orientation.w = left_foot_rot.w();
    left_foot_marker.scale.x = 0.09;
    left_foot_marker.scale.y = 0.27;
    left_foot_marker.scale.z = 0.01;
    left_foot_marker.color.r = 35.0/255.0;
    left_foot_marker.color.g = 148.0/255.0;
    left_foot_marker.color.b = 140.0/255.0;
    left_foot_marker.color.a = 1.0;

    visualization_msgs::Marker right_foot_marker;
    right_foot_marker.header.frame_id = global_frame_id;
    right_foot_marker.header.stamp = ros::Time::now();
    right_foot_marker.ns = "right_foot_" + std::to_string(hidx);
    right_foot_marker.id = 1;
    right_foot_marker.type = visualization_msgs::Marker::CUBE;
    right_foot_marker.action = visualization_msgs::Marker::ADD;
    right_foot_marker.pose.position.x = right_foot_pos.x();
    right_foot_marker.pose.position.y = right_foot_pos.y();
    right_foot_marker.pose.position.z = right_foot_pos.z() + 0.05;
    right_foot_marker.pose.orientation.x = right_foot_rot.x();
    right_foot_marker.pose.orientation.y = right_foot_rot.y();
    right_foot_marker.pose.orientation.z = right_foot_rot.z();
    right_foot_marker.pose.orientation.w = right_foot_rot.w();
    right_foot_marker.scale.x = 0.09;
    right_foot_marker.scale.y = 0.27;
    right_foot_marker.scale.z = 0.01;
    right_foot_marker.color.r = 17.0/255.0;
    right_foot_marker.color.g = 125.0/255.0;
    right_foot_marker.color.b = 90.0/255.0;
    right_foot_marker.color.a = 1.0;

    markers.markers.push_back(left_foot_marker);
    markers.markers.push_back(right_foot_marker);
    expansions_pub.publish(markers);

    ros::Duration(0.002).sleep();
}


void visualize_hbsp_heuristic_values(
    const ros::NodeHandle& nh,
    const std::string& global_frame_id,
    const std::shared_ptr<smpl::SparseDistanceMap> distance_map,
    const std::shared_ptr<environment::EnvironmentInterpreter>
        environment_interpreter,
    const std::shared_ptr<graphs::NavLattice2D> environment,
    const std::shared_ptr<planners::HBSP> hbsp,
    const ros::Publisher& h_vals_pub) {
    std::map<std::string, double> params;
    nh.getParam("map_params", params);

    visualization_msgs::MarkerArray markers;
    const double m_scale = params["cell_size"];
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = global_frame_id;
    marker.ns = "HVALS";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = m_scale;
    marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0f;

    // maximum, non-infinite, heuristic value, for colorization
    const double max_cost = hbsp->get_hbsp_max_cost();

    // pre-compute colors
    std::vector<std_msgs::ColorRGBA> hue_color_map(360);
    leatherman::msgHSVToRGB(0.0, 1.0, 1.0, hue_color_map[0]);
    for (int i = 120; i < 360; ++i) {
        leatherman::msgHSVToRGB(
            static_cast<double>(i), 1.0, 1.0, hue_color_map[i]);
    }

    const double max_cost_inv = 1.0 / max_cost;

    std::vector<std::vector< int > > signatures;
    for (auto& x : hbsp->get_suffixes()) {
        signatures.push_back(x);
    }
    signatures.push_back({});
    const planners::VertexCostMap cost_map = hbsp->get_hbsp_cost_map();

    const auto env_projections =
        environment_interpreter->get_environment_projections();

    const int rows = distance_map->numCellsY();
    const int cols = distance_map->numCellsX();

    for (int i = 0; i < env_projections->projections_size(); ++i) {
        const environment::proto::EnvironmentProjection projection =
            env_projections->projections(i);

        for (int x = 0; x < cols; ++x) {
            for (int y = 0; y < rows; ++y) {
                int id = environment->get_state_id(x, y, i);
                double min_cost = std::numeric_limits<double>::max();
                double curr_cost = std::numeric_limits<double>::max();
                for (auto& signature : signatures) {
                    graphs::Vertex vertex = {
                        .id = id,
                        .signature_id =
                            environment->get_signature_id(signature)};
                    auto vertex_cost_pair = cost_map.find(vertex);
                    if (vertex_cost_pair == cost_map.end()) {
                        continue;
                    }

                    curr_cost = vertex_cost_pair->second;
                    if (curr_cost < min_cost) {
                        min_cost = curr_cost;
                    }
                }

                if (min_cost > max_cost) {
                    continue;
                }

                const int surface_2d_idx = (y * cols) + x;
                const int surface_z =
                    projection.workspace_2d_to_surface_3d(surface_2d_idx);

                const int surface_3d_idx =
                    surface_z * rows * cols + surface_2d_idx;
                const int stepping_z =
                    env_projections->surface_3d_to_stepping_cells(
                        surface_3d_idx);

                double wx, wy, wz;
                distance_map->gridToWorld(x, y, stepping_z, wx, wy, wz);
                geometry_msgs::Point p;
                p.x = wx; p.y = wy; p.z = wz;
                marker.points.push_back(p);

                const double alpha = min_cost * max_cost_inv;
                int hue = static_cast<int>((120.0 + alpha * 240.0)) % 360;
                const std_msgs::ColorRGBA &col = hue_color_map[hue];
                marker.colors.push_back(col);
            }
        }
    }

    ROS_INFO_NAMED("HomotopicBasedHeuristic",
        "Visualizing %zu homotopic_based_heuristic points",
        marker.points.size());
    markers.markers.push_back(marker);

    h_vals_pub.publish(markers);
}

void visualize_path(
    const std::vector<int>& solution_stateIDs_V,
    const std::shared_ptr<graphs::NavLattice8D> environment,
    const std::string& global_frame_id,
    const ros::Publisher& path_pub) {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = global_frame_id;
    marker.ns = "final path";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = 0.1;
    marker.color.r = marker.color.g = marker.color.b = 0.0;
    marker.color.a = 1.0f;

    std_msgs::ColorRGBA color_l;
    color_l.r = 35.0/255.0;
    color_l.g = 148.0/255.0;
    color_l.b = 140.0/255.0;
    color_l.a = 1.0;

    std_msgs::ColorRGBA color_r;
    color_r.r = 17.0/255.0;
    color_r.g = 125.0/255.0;
    color_r.b = 90.0/255.0;
    color_r.a = 1.0;

    for (const int state_id : solution_stateIDs_V) {
        const graphs::BipedalState* bipedal_state =
            environment->get_bipedal_state(state_id);

        const Eigen::Vector4d left_foot_pos =
            environment->get_continuous_coordinates(
                bipedal_state->left_foot_id);

        geometry_msgs::Point p_l;
        p_l.x = left_foot_pos.x();
        p_l.y = left_foot_pos.y();
        p_l.z = left_foot_pos.z() + 0.05;
        marker.points.push_back(p_l);
        marker.colors.push_back(color_l);

        const Eigen::Vector4d right_foot_pos =
        environment->get_continuous_coordinates(
            bipedal_state->right_foot_id);

        geometry_msgs::Point p_r;
        p_r.x = right_foot_pos.x();
        p_r.y = right_foot_pos.y();
        p_r.z = right_foot_pos.z() + 0.05;
        marker.points.push_back(p_r);
        marker.colors.push_back(color_r);
    }

    path_pub.publish(marker);
}

}  // namespace visualize
}  // namespace footstep_planner
