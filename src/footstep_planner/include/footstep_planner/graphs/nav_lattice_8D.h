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

#ifndef SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_GRAPHS_NAV_LATTICE_8D_H_
#define SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_GRAPHS_NAV_LATTICE_8D_H_

#include <footstep_planner/graphs/homotopy_information.h>
#include <footstep_planner/utils/datatypes.h>
#include <robot_parameters.pb.h>
#include <environment_projections.pb.h>
#include <smpl/distance_map/sparse_distance_map.h>
#include <vector>
#include <unordered_map>

namespace footstep_planner {
namespace graphs {

// This class implements an 8D navigation lattice that is used for footstep
// planning for a humanoid robot. This navigation lattice is also augmented
// with homotopy information (i.e. the h-signatures)
//
// There are two types of states: (1) BipedalState and (2) FootState. The
// BiepdalState is an 8D h-augmented state and the FootState is a 4D state that
// defines the state of a single foot. The BipedalState is comprised of two
// FootStates (see utils/datatypes.h).
class NavLattice8D {
 public:
    // The navigation lattice is constructed using the following:
    // (1) distance_map Contains obstacle information, more specifically the
    // distance from each cell to the nearest obstacle
    // (2) robot_details The protobuf containing robot-specific information
    // (see proto/robot_parameters/robot_parameters.proto)
    // (3) homotopy_information Used for calculating homotopy information on a
    // state in the graph
    // (4) valid_stepping_cells Contains information on the specific cells that
    // that the robot can place its foot
    NavLattice8D(
        const std::shared_ptr<smpl::SparseDistanceMap> distance_map,
        const proto::RobotParameters& robot_details,
        const std::shared_ptr<graphs::HomotopyInformation> homotopy_information,
        const std::shared_ptr<environment::proto::ValidSteppingCells>
            valid_stepping_cells);

    ~NavLattice8D();

    // Given the coordinates of the point between the feet, the left and right
    // foot states are extracted to create the bipedal state
    int set_bipedal_state(
        const int& x,
        const int& y,
        const int& z,
        const int& theta);

    // Creates the start state for the given position. If the start state is not
    // valid throws a ROS_ERROR and returns the start state id otherwise
    int set_start_state(
        const int& x,
        const int& y,
        const int& z,
        const double& theta_rad);

    // Create the goal state for the given position. If the goal state is not
    // valid throws a ROS_ERROR and returns the goal state id otherwise
    int set_goal_state(
        const int& x,
        const int& y,
        const int& z,
        const double& theta_rad);

    // Returns the foot state at the given ID, if it exists, and NULL otherwise
    FootState* get_foot_state(const int& state_id) const;

    // Returns the foot state at the give position vector, if it exists, and
    // NULL otherwise
    FootState* get_foot_state(const Eigen::Vector4d& state_pos) const;

    // Returns the foot state at the given discretized position values, if it
    // exists, and NULL otherwise
    FootState* get_foot_state(
        const int& x,
        const int& y,
        const int& z,
        const int& theta) const;

    // Returns the foot state at the give continuous position values
    // (meters/radians), if it exists, and NULL otherwise
    FootState* get_foot_state(
        const double& x_m,
        const double& y_m,
        const double& z_m,
        const double& theta_rad) const;

    // Returns the bipedal state at the state ID, if it exists, and NULL
    // otherwise
    BipedalState* get_bipedal_state(const int& state_id);

    // Returns true if the bipedal state is valid and false otherwise
    bool is_valid_bipedal_state(const int& state_id);

    // Given a discretized position vector, returns true if the foot state is
    // valid and false otherwise
    bool is_valid_foot_state(const Eigen::Vector3i& foot_pos);

    // Given discretized position values, returns true if the foot state is
    // valid and false otherwise
    bool is_valid_foot_state(
        const int& x,
        const int& y,
        const int& z);

    // Given a source state id, this function fills in valid successor biepdal
    // state IDs and their associated costs into succ_ids and costs respectively
    void get_succs(
        const int& source_state_id,
        std::vector<int>* succ_ids,
        std::vector<double>* costs);

    // Returns true if the given state is within a specified distance from the
    // goal (goal tolerance defined in robot_parameters.proto)
    bool is_goal(const int& current_state_id);

    // Returns the state ID given discretized coordinates
    int get_foot_state_id(
        const int& x,
        const int& y,
        const int& z,
        const int& theta) const;

    // Returns the state ID given continuous coordinates in meters/radians
    int get_foot_state_id(
        const double& x_m,
        const double& y_m,
        const double& z_m,
        const double& theta_rad) const;

    // Returns a vector of continous coordinates for the given state ID
    Eigen::Vector4d get_continuous_coordinates(const int& state_4D_id) const;

    // Fills in vector of transformed collision spheres based on the current
    // bipedal state; the vector contains coordinates in the following format:
    // (origin_x_m, origin_y_m, origin_z_m, radius_m)
    //
    // See proto/robot_parameters/robot_parameters.proto for collision sphere
    // definition
    void get_transformed_collision_spheres(
        const int &bipedal_state_id,
        std::vector<Eigen::Vector4d>* transformed_spheres);

    // Returns a vector containing the continuous values (meters/radians) of the
    // point between the feet
    Eigen::Vector4d get_cont_averaged_state(
        const int& left_state_id,
        const int& right_state_id) const;

    // Returns a vector containing the discretized values of the point between
    // the feet
    Eigen::Vector4i get_disc_averaged_state(
        const int& left_state_id,
        const int& right_state_id) const;

    // Given the signature ID, fills in the signature vector
    void get_signature(
        const int& signature_id,
        std::vector<int>* signature);

 private:
    // This function finds the successors for each foot and fills in the vector
    // with valid foot IDs. The foot can only expand in the forward direction;
    // the magnitude of the movement is defined by the motion primitives
    //
    // See proto/robot_parameters/robot_parameters.proto for motion primitive
    // definition
    void get_foot_succs(
        const BipedalState& bipedal_state,
        std::vector<int>* foot_succs);

    // Creates a state at the specified continuous coordinates if one does not
    // exists and returns the newly created state
    FootState* create_new_foot_state(const Eigen::Vector4d& state_pos);

    // Creates a state at the specified discretized coordinates if one does not
    // exists and returns the newly created state
    FootState* create_new_foot_state(
        const int& x,
        const int& y,
        const int& z,
        const int& theta);

    // Creates a state at the specified continuous coordinates if one does not
    // exists and returns the newly created state
    FootState* create_new_foot_state(
        const double& x_m,
        const double& y_m,
        const double& z_m,
        const double& theta_rad);

    // Creates a new bipedal state and returns its state ID
    int create_new_bipedal_state(const BipedalState& new_state);

    // Returns the cost of executing the action to move from the source state
    // to the new state.
    double get_action_cost(
        const FootState* source_state,
        const FootState* new_state);

    // Returns the ID for the active foot (i.e. foot state that is going to be
    // expanded); The next_foot value in the struct defines which foot state is
    // going to be expanded
    int get_active_foot_id(const BipedalState& bipedal_state);

    // Returns the ID for the pivot foot (i.e. the foot state that is not going
    // to be expanded)
    int get_pivot_foot_id(const BipedalState& bipedal_state);

    // Return the ID for the given signature
    int get_signature_id(const std::vector<int>& signature);

    // Creates the ID for the given signature and returns it
    int create_new_signature(const std::vector<int>& signature);

    // The goal state; note that this state contains the coordinates for the
    // point between the feet
    FootState* goal_state_;

    // A mapping between state IDs and state coordinates
    std::unordered_map<int, FootState*> foot_ID_to_state_;

    // A mapping between a signature and its ID and vice versa
    std::unordered_map<std::vector<int>, int> signature_to_ID_;
    std::vector<std::vector<int>> ID_to_signature_;

    // A mapping between a bipedal state and its ID and ivce versa
    std::unordered_map<BipedalState, int> bipedal_state_to_ID_;
    std::vector<BipedalState*> bipedal_ID_to_state_;

    // The distance map of the environment
    std::shared_ptr<smpl::SparseDistanceMap> distance_map_;

    // The unpacked robot parameters protobuf
    robot_details::RobotParameters robot_parameters_;

    const std::shared_ptr<graphs::HomotopyInformation> homotopy_information_;

    const std::shared_ptr<environment::proto::ValidSteppingCells>
        valid_stepping_cells_;
};

}  // namespace graphs
}  // namespace footstep_planner

#endif  // SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_GRAPHS_NAV_LATTICE_8D_H_
