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
int main(int argc, char* argv[]) {

    ROS_INFO("project_environment");

    ros::init(argc, argv, "project_environment");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");


    std::string global_frame_id = "world";

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
    footstep_planner::proto::RobotParameters robot_details;
    google::protobuf::TextFormat::Parse(&file_input, &robot_details);

    std::map<std::string, double> map_params;
    nh.getParam("map_params", map_params);

    const auto environment_interpreter =
        std::make_shared<footstep_planner::environment::EnvironmentInterpreter>(
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

    if (!environment_interpreter->project_and_save_environment(
        projections_file.c_str(),
        stepping_cells_file.c_str())) return 1;

    ROS_INFO("Saved projections message");

    // Initialize and save homotopy information for the projected environment
    const std::string beams_outfile =
        footstep_planner_path + proto_msgs["beams"];
    const std::string gates_outfile =
        footstep_planner_path + proto_msgs["gates"];
    const auto homotopy_information =
        std::make_shared<footstep_planner::graphs::HomotopyInformation>(
            environment_interpreter->get_environment_projections());

    if (!homotopy_information->find_and_save_beams_and_gates(
        beams_outfile,
        gates_outfile)) return 1;

    ROS_INFO("Saved beams and gates");

    return 0;
}
