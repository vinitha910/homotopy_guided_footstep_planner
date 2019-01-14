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

#ifndef SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_GRAPHS_NAV_LATTICE_2D_H_
#define SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_GRAPHS_NAV_LATTICE_2D_H_

#include <footstep_planner/utils/helpers.h>
#include <footstep_planner/utils/datatypes.h>
#include <footstep_planner/proto/environment_projections/unpack_from_proto.h>
#include <environment_projections.pb.h>
#include <Eigen/Geometry>
#include <memory>
#include <vector>
#include <unordered_map>
#include <utility>

namespace footstep_planner {
namespace graphs {

// This class implements a 2D navigation lattice given an EnvironmentProjections
// protobuf (see proto/environment_projections/environment_projections.proto)
class NavLattice2D {
 public:
    NavLattice2D(
        const std::shared_ptr<environment::proto::EnvironmentProjections>
        env_projections);

    ~NavLattice2D();

    // Returns the state ID
    int get_state_id(const int& x, const int& y, const int& workspace);

    // Gets the coordinates for the given state ID
    bool get_coord_from_state_id(
        const int& state_id,
        int* x,
        int* y,
        int* workspace);

    // Returns true if the state is valid and false otherwise
    bool is_valid_state(const int &x, const int &y, const int &workspace);

    // Sets the start state for the graph and returns the corresponding ID
    int set_start_state(
        const int& x,
        const int& y,
        const int& workspace);

    // Sets the goal state for the graphs and returns the corresponding ID
    int set_goal_state(
        const int& x,
        const int& y,
        const int& workspace);

    // Given ID for the source state, this function fills the succ_ids vector
    // with valid successor IDs and the costs vector with their associated cost
    void get_succs(
        const int& source_state_id,
        std::vector<int> *succ_ids,
        std::vector<int> *costs);

    // This function prints the state
    void print_state(const int& state_id, const double& cost);

    // Returns true if the given ID is equal to the start state ID
    bool is_start_state(const int& state_id) { return state_id == start_id_; }

    // Given an (x, y, workspace) this function finds another workspace that
    // contains the given (x, y). If not such workspace exists, the function
    // returns the given workspace
    int find_connected_workspace(
        const int& x,
        const int& y,
        const int& workspace);

    // Returns the ID of the signature given. If not such ID exists, this
    // functions assigns the signature an ID and returns it
    int find_or_create_signature(const std::vector<int>& signature);

    // Given the ID of a signature, this function fills in the signature vector
    void get_signature(
        const int& signature_id,
        std::vector<int>* signature);

    // Returns the ID associated with the given signature
    int get_signature_id(const std::vector<int>& signature);

 private:
    // Creates a new ID for the signature and returns the ID
    int create_new_signature(const std::vector<int>& signature);

    // Checks the validity of the new state, creates the state if it's valid
    // and adds the state and its cost to the succ_ids vector and costs vector
    // respectively
    void add_successor(
        std::vector<int> *succ_ids,
        std::vector<int> *costs,
        const int& x,
        const int& y,
        const int& workspace,
        const int& new_x,
        const int& new_y,
        const int& new_workspace);

    // Searches for a state at the specified coordinates;
    // Returns the state if it exists and NULL otherwise
    State2D* get_state(const int& x, const int& y, const int& workspace);

    // Creates a state at the specified coordinates if one does not exist;
    // Returns the newly created state
    State2D* create_new_state(const int& x, const int& y, const int& workspace);

    // Returns the cost of executing the action to move from the source state
    // to the new state.
    int get_action_cost(
        const int &source_x,
        const int &source_y,
        const int &source_workspace,
        const int &new_x,
        const int &new_y,
        const int &new_workspace);

    // Mappings to maps the signature to its ID and vice versa
    std::unordered_map<std::vector<int>, int> signature_to_ID_;
    std::vector<std::vector<int>> ID_to_signature_;

    // A mapping between state IDs and state coordinates
    std::vector<State2D*> state_ID_to_coord_map_;

    // The motion primitives used for finding successors
    std::vector<std::pair<int, int>> mprims_;

    // A mapping between (x, y, surface_z) points and workspace indicies'
    // (i.e. (lower_workspace_idx, upper_workspace_idx))
    std::vector<std::pair<int, int>> surface_3d_to_indices_;

    // A vector of projections
    std::vector<Projection> projections_;

    // The number of rows and columns in each projection
    int num_rows_;
    int num_cols_;

    // The start and goal ID
    int start_id_;
    int goal_id_;
};

}  // namespace graphs
}  // namespace footstep_planner

#endif  // SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_GRAPHS_NAV_LATTICE_2D_H_
