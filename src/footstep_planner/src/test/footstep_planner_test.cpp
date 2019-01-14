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

#include <footstep_planner/graphs/homotopy_information.h>
#include <footstep_planner/graphs/nav_lattice_2D.h>
#include <footstep_planner/graphs/nav_lattice_8D.h>
#include <footstep_planner/planners/hbsp.h>
#include <footstep_planner/planners/dijkstra.h>
#include <footstep_planner/planners/mha.h>
#include <footstep_planner/heuristics/homotopy_based_heuristic.h>
#include <footstep_planner/heuristics/backward_dijkstra_heuristic.h>
#include <footstep_planner/rviz/visualize.h>
#include <fcntl.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <fstream>

using namespace footstep_planner;

int get_closest_stepping_cell(
    const int x, const int y, const int surf,
    const std::shared_ptr<environment::proto::EnvironmentProjections>
        environment) {
    const int num_cells_x = environment->cols();
    const int num_cells_y = environment->rows();
    const int index_2d = (y * num_cells_x) + x;
    const int surf_z =
        environment->projections(surf).workspace_2d_to_surface_3d(index_2d);
    const int index_3d = surf_z * num_cells_y * num_cells_x + index_2d;
    return environment->surface_3d_to_stepping_cells(index_3d);
}

bool read_signature_array(
    XmlRpc::XmlRpcValue& signatures_xml,
    std::vector<std::vector<int>>* signatures) {
    if (signatures_xml.size() > 0) {
        for (int i = 0; i < signatures_xml.size(); ++i) {
            XmlRpc::XmlRpcValue& signature_xml = signatures_xml[i];
            if (signature_xml.getType() != XmlRpc::XmlRpcValue::TypeArray) {
                ROS_ERROR("Signature must be represented as an array");
                return false;
            }

            std::vector<int> signature;
            for (int j = 0; j < signature_xml.size(); ++j) {
                XmlRpc::XmlRpcValue& beam = signature_xml[j];
                if (beam.getType() != XmlRpc::XmlRpcValue::TypeInt) {
                    ROS_ERROR("Signature must be an array of integers");
                    return false;
                }
                signature.push_back(std::move((static_cast<int>(beam))));
            }
            signatures->push_back(signature);
        }
    }
}

bool read_signatures(
    std::vector<std::vector<int>>* signatures,
    std::vector<std::vector<int>>* h_signatures,
    const ros::NodeHandle& nh) {
    XmlRpc::XmlRpcValue signatures_xml;
    nh.getParam("signatures", signatures_xml);
    if (signatures_xml.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("signatures parameter must be an array");
        return false;
    }
    signatures->reserve(signatures_xml.size());
    read_signature_array(signatures_xml, signatures);

    XmlRpc::XmlRpcValue h_signatures_xml;
    nh.getParam("h_signatures", h_signatures_xml);
    if (h_signatures_xml.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("h_signatures parameter must be an array");
        return false;
    }
    h_signatures->reserve(h_signatures_xml.size());
    read_signature_array(h_signatures_xml, h_signatures);

    return true;
}

int main(int argc, char* argv[]) {
    ROS_INFO("footstep_planner_test");

    ros::init(argc, argv, "footstep_local_planner_test");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    const char *stats_filepath = nullptr;
    const char *test_num = nullptr;
    if (argc > 1) {
        stats_filepath = argv[1];
        ROS_INFO("Append statistics to %s", stats_filepath);
        test_num = argv[2];
        ROS_INFO("Test #%s", test_num);
    }

    ros::Publisher obstacles_vis_pub =
        nh.advertise<visualization_msgs::Marker>("obstacles_marker", 1);
    ros::Publisher platforms_vis_pub =
        nh.advertise<visualization_msgs::Marker>("platforms_marker", 1);
    ros::Publisher h_vals_pub =
        nh.advertise<visualization_msgs::MarkerArray>("heuristic_vals", 1);
    ros::Publisher goal_pub =
        nh.advertise<visualization_msgs::Marker>("goal_region", 1);
    ros::Publisher start_pub =
        nh.advertise<visualization_msgs::Marker>("start_region", 1);
    ros::Publisher path_pub =
        nh.advertise<visualization_msgs::Marker>("path", 1);

    std::string global_frame_id = "world";

    visualize::publish_child_frame(nh, global_frame_id, "obstacles_marker");
    visualize::publish_child_frame(nh, global_frame_id, "platforms_marker");
    visualize::publish_child_frame(nh, global_frame_id, "heuristic_vals");
    visualize::publish_child_frame(nh, global_frame_id, "goal_region");
    visualize::publish_child_frame(nh, global_frame_id, "start_region");
    visualize::publish_child_frame(nh, global_frame_id, "path");

    // Read in parameters giving stl paths
    std::string platforms;
    std::string obstacles;
    std::string query_type;
    std::vector<std::string> surfaces;
    std::vector<std::string> stepping_surfaces;
    nh.getParam("surfaces", surfaces);
    nh.getParam("platforms", platforms);
    nh.getParam("query_type", query_type);
    nh.getParam("obstacles", obstacles);
    nh.getParam("stepping_surfaces", stepping_surfaces);

    std::map<std::string, std::string> proto_msgs;
    nh.getParam("proto_msgs", proto_msgs);
    const std::string footstep_planner_path =
        ros::package::getPath(proto_msgs["package"]);

    // Parses robot parameters proto message
    std::string robot_details_path =
        footstep_planner_path + proto_msgs["robot_parameters"];
    int robot_details_file = open(robot_details_path.c_str(), O_RDONLY);
    google::protobuf::io::FileInputStream file_input(robot_details_file);
    proto::RobotParameters robot_details;
    google::protobuf::TextFormat::Parse(&file_input, &robot_details);

    std::map<std::string, double> map_params;
    nh.getParam("map_params", map_params);

    const auto environment_interpreter =
        std::make_shared<environment::EnvironmentInterpreter>(
            platforms,
            obstacles,
            surfaces,
            stepping_surfaces,
            map_params,
            robot_details);

    // Project and save environment
    const std::string projections_file =
        footstep_planner_path + proto_msgs["projections"];
    const std::string stepping_cells_file =
        footstep_planner_path + proto_msgs["stepping_cells"];

    bool load_proto_msgs;
    nh.getParam("load_proto_msgs", load_proto_msgs);

    if (!load_proto_msgs) {
        if (!environment_interpreter->project_and_save_environment(
            projections_file.c_str(),
            stepping_cells_file.c_str())) return 1;
    } else {
        ROS_INFO("Parsing Environment Projections...");
        if (!environment_interpreter->read_projection(
            projections_file.c_str(),
            stepping_cells_file.c_str())) return 1;
    }

    // Initialize and save homotopy information for the projected environment
    const std::string beams_outfile =
        footstep_planner_path + proto_msgs["beams"];
    const std::string gates_outfile =
        footstep_planner_path + proto_msgs["gates"];
    const auto homotopy_information =
        std::make_shared<graphs::HomotopyInformation>(
            environment_interpreter->get_environment_projections());
    if (!load_proto_msgs) {
        if (!homotopy_information->find_and_save_beams_and_gates(
            beams_outfile,
            gates_outfile)) return 1;
    } else {
        ROS_INFO("Parsing Beams and Gates...");
        if (!homotopy_information->read_beams_and_gates(
            beams_outfile,
            gates_outfile)) return 1;
    }

    // Initialize graph for the projected environment
    const auto nav_lattice_2d = std::make_shared<graphs::NavLattice2D>(
        environment_interpreter->get_environment_projections());

    std::map<std::string, double> goal;
    nh.getParam("goal_pose", goal);

    // The robot goal state is the start state for the heuristic
    const int projected_start_id =
        nav_lattice_2d->set_start_state(goal["x"], goal["y"], goal["workspace"]);
    if (projected_start_id == -1) {
        ROS_ERROR("Error: Invalid Start Pose!");
        return 1;
    }

    // Get start and goal IDs for heuristics
    std::map<std::string, double> start;
    nh.getParam("start_pose", start);

    // The robot start state is the goal state for the heuristic
    const int projected_goal_id =
        nav_lattice_2d->set_goal_state(
            start["x"], start["y"], start["workspace"]);
    if (projected_goal_id == -1) {
        ROS_ERROR("Error: Invalid Goal Pose!");
        return 1;
    }

    // Read the signatures from the scenario file
    std::vector<std::vector<int>> signatures;
    std::vector<std::vector<int>> h_signatures;
    read_signatures(&signatures, &h_signatures, nh);

    ROS_INFO("Generating anchor heuristic...");
    double start_time =
        static_cast<double>(clock()) / static_cast<double>(CLOCKS_PER_SEC);

    // Run planner for the 2D Backwards Dijkstra Heuristic
    const auto dijkstra_planner =
         std::make_shared<planners::Dijkstra>(
            nav_lattice_2d,
            projected_start_id);
    double end_time =
        static_cast<double>(clock()) / static_cast<double>(CLOCKS_PER_SEC);
    const double anchor_heuristic_gen_time = end_time - start_time;

    // Create the 2D Backwards Dijkstra Heuristic
    const auto dijkstra_heur =
        std::make_shared<heuristics::BackwardDijkstraHeuristic>(
            dijkstra_planner,
            environment_interpreter->get_environment_projections());

    ROS_INFO("Generating homotopy-based heuristics...");
    start_time =
        static_cast<double>(clock()) / static_cast<double>(CLOCKS_PER_SEC);
    // Run planner for hbsp heuristic
    const auto hbsp_planner =
        std::make_shared<planners::HBSP>(
            homotopy_information,
            nav_lattice_2d,
            projected_goal_id,
            projected_start_id,
            signatures,
            h_signatures);
    end_time =
        static_cast<double>(clock()) / static_cast<double>(CLOCKS_PER_SEC);
    const double hbsp_heuristic_gen_time =
        signatures.size() > 0 ? end_time - start_time : 0.0;

    // Create the HBSP Heuristics
    const auto hbsp_heur =
        std::make_shared<heuristics::HomotopyBasedHeuristic>(
            hbsp_planner,
            environment_interpreter->get_environment_projections());

    const auto nav_lattice_4d = std::make_shared<graphs::NavLattice8D>(
        environment_interpreter->get_distance_map(),
        robot_details,
        homotopy_information,
        environment_interpreter->get_valid_stepping_cells());

    const int start_z =
        get_closest_stepping_cell(
            start["x"],
            start["y"],
            start["workspace"],
            environment_interpreter->get_environment_projections());
    const int start_state_id =
        nav_lattice_4d->set_start_state(
            start["x"],
            start["y"],
            start_z,
            start["theta"]);
     if (start_state_id == -1) {
        ROS_ERROR("Error: Invalid Robot Start Pose.");
        return 1;
    }

    const int goal_z =
        get_closest_stepping_cell(
            goal["x"],
            goal["y"],
            goal["workspace"],
            environment_interpreter->get_environment_projections());
    const int goal_state_id =
        nav_lattice_4d->set_goal_state(
            goal["x"],
            goal["y"],
            goal_z,
            goal["theta"]);
    if (goal_state_id == -1) {
        ROS_ERROR("Error: Invalid Robot Goal Pose.");
        return 1;
    }

    const auto mha_planner =
        std::make_shared<planners::MHAPlanner>(
            nav_lattice_4d,
            dijkstra_heur,
            hbsp_heur,
            signatures.size(),
            nh,
            global_frame_id);

    if (mha_planner->set_start(start_state_id) == 0) {
        return 1;
    }
    if (mha_planner->set_goal(goal_state_id) == 0) {
        return 1;
    }

    std::map<std::string, bool> visualize;
    nh.getParam("visualize", visualize);

    if (visualize["robot_model"]) {
        double x_m, y_m, z_m;
        environment_interpreter->get_distance_map()->gridToWorld(
            start["x"], start["y"], start_z, x_m, y_m, z_m);
        visualize::visualize_robot_model(
            nh, global_frame_id, x_m, y_m, z_m, start["theta"]);

        environment_interpreter->get_distance_map()->gridToWorld(
            goal["x"], goal["y"], goal_z, x_m, y_m, z_m);
        visualize::visualize_goal_region(
            nh, global_frame_id, goal_pub, x_m, y_m, z_m);
    }

    if (visualize["world"]) {
        visualize::visualize_obstacles(
            obstacles, global_frame_id, obstacles_vis_pub);

        visualize::visualize_platforms(
            platforms, global_frame_id, platforms_vis_pub);
    }

    if (visualize["homotopic_heuristic"]) {
        visualize::visualize_hbsp_heuristic_values(
            nh,
            global_frame_id,
            environment_interpreter->get_distance_map(),
            environment_interpreter,
            nav_lattice_2d,
            hbsp_planner,
            h_vals_pub);
    }

    if (visualize["anchor_heuristic"]) {
        visualize::visualize_dijkstra_heuristic_values(
              nh,
              global_frame_id,
              environment_interpreter->get_distance_map(),
              environment_interpreter,
              nav_lattice_2d,
              dijkstra_planner,
              h_vals_pub);
    }

    std::vector<int> solution_stateIDs_V;
    int solcost;

    ROS_INFO("Planning...");
    const int sol = mha_planner->replan(60.0, &solution_stateIDs_V, &solcost);

    const double total_planning_time =
        hbsp_heuristic_gen_time + \
        anchor_heuristic_gen_time + \
        mha_planner->get_final_eps_planning_time();

    // log statistics if stats file given on command line
    if (stats_filepath != nullptr) {
        FILE* file = fopen(stats_filepath, "a");
        if (file) {
            fprintf(file, "\n");
            fprintf(file, "test_%s:\n", test_num);
            fprintf(file, "    solution_found:        %d\n", sol);
            fprintf(file, "    hbsp_heuristic_time:   %f\n", hbsp_heuristic_gen_time);
            fprintf(file, "    anchor_heuristic_time: %f\n", anchor_heuristic_gen_time);
            fprintf(file, "    planning_time:         %f\n", mha_planner->get_final_eps_planning_time());
            fprintf(file, "    total_planning_time:   %f\n", total_planning_time);
            fprintf(file, "    path length:           %d\n", static_cast<int>(solution_stateIDs_V.size()));
            fprintf(file, "    query_type:            \"%s\"\n\n", query_type.c_str());
            fclose(file);
        } else {
            ROS_WARN("Failed to append stats to %s", stats_filepath);
        }
    }

    // print statistics
    ROS_INFO("Statistics:");
    ROS_INFO("    Solution Cost: %d", solcost);
    ROS_INFO("    hbsp_heuristic_time:      %f", hbsp_heuristic_gen_time);
    ROS_INFO("    anchor_heuristic_time:    %f", anchor_heuristic_gen_time);
    ROS_INFO("    planning time:            %f", mha_planner->get_final_eps_planning_time());
    ROS_INFO("    total_planning_time:      %f", total_planning_time);
    ROS_INFO("    path length:              %d", static_cast<int>(solution_stateIDs_V.size()));

    if (visualize["path"]) {
        visualize::visualize_path(
            solution_stateIDs_V,
            nav_lattice_4d,
            global_frame_id,
            path_pub);
    }

    // Uncomment if visualizing
    // ros::spin();
    return 0;
}
