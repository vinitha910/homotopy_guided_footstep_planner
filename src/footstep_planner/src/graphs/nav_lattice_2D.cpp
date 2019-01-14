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

#include <footstep_planner/graphs/nav_lattice_2D.h>
#include <ros/ros.h>

namespace footstep_planner {
namespace graphs {

NavLattice2D::NavLattice2D(
    const std::shared_ptr<environment::proto::EnvironmentProjections>
    env_projections) {
    unpack_from_proto(&num_rows_, &num_cols_, &projections_, env_projections);
    unpack_from_proto(&surface_3d_to_indices_, env_projections);

    // Creates the motion primitives for an 8-connected grid
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            // Ignore no motion
            if (dx == 0 && dy == 0) continue;

            mprims_.push_back(std::make_pair(dx, dy));
        }
    }
    state_ID_to_coord_map_ =
        std::vector<State2D*>(
            projections_.size()*num_rows_*num_cols_,
            nullptr);
}

NavLattice2D::~NavLattice2D() {
    // free states
    for (size_t i = 0; i < state_ID_to_coord_map_.size(); ++i) {
        // free search state
        delete (state_ID_to_coord_map_[i]);
    }
    state_ID_to_coord_map_.clear();
}

bool NavLattice2D::get_coord_from_state_id(
    const int& state_id,
    int* x,
    int* y,
    int* workspace) {
    if (state_id >= state_ID_to_coord_map_.size()) {
        return false;
    }

    const auto state = state_ID_to_coord_map_[state_id];
    *x = state->x;
    *y = state->y;
    *workspace = state->workspace;
    return true;
}

int NavLattice2D::find_connected_workspace(
    const int& x,
    const int& y,
    const int& workspace) {
    const int proj_state_idx = (y * num_cols_) + x;
    const int z = projections_[workspace].workspace_2d_to_surface_3d[proj_state_idx];
    const int state_idx = (z * num_rows_ * num_cols_) + proj_state_idx;
    if (surface_3d_to_indices_[state_idx].second >= 0) {
        const int lower_workspace = surface_3d_to_indices_[state_idx].first;
        const int upper_workspace = surface_3d_to_indices_[state_idx].second;
        return (workspace != lower_workspace ? lower_workspace : upper_workspace);
    }

    return workspace;
}

int NavLattice2D::get_state_id(
    const int& x,
    const int& y,
    const int& workspace) {
    return (x + (y * num_cols_) + (workspace * num_rows_ * num_cols_));
}

State2D* NavLattice2D::get_state(
    const int& x,
    const int& y,
    const int& workspace) {
    assert(workspace < projections_.size());

    const int state_id = get_state_id(x, y, workspace);
    if (state_id >= state_ID_to_coord_map_.size()) {
        return NULL;
    }

    return state_ID_to_coord_map_[state_id];
}

State2D* NavLattice2D::create_new_state(
    const int& x,
    const int& y,
    const int& workspace) {
    const int state_id = get_state_id(x, y, workspace);

    state_ID_to_coord_map_[state_id] = new State2D;
    state_ID_to_coord_map_[state_id]->x = x;
    state_ID_to_coord_map_[state_id]->y = y;
    state_ID_to_coord_map_[state_id]->workspace = workspace;

    return state_ID_to_coord_map_[state_id];
}

bool NavLattice2D::is_valid_state(
    const int &x,
    const int &y,
    const int &workspace) {
    assert(workspace < projections_.size());

    const int workspace_2d_idx = x + (y * num_cols_);

    const environment::CellType cell =
        projections_[workspace].data[workspace_2d_idx];
    return (cell == environment::FREE_CELL || cell == environment::GATE_CELL);
}

int NavLattice2D::get_action_cost(
    const int &source_x,
    const int &source_y,
    const int &source_workspace,
    const int &new_x,
    const int &new_y,
    const int &new_workspace) {
    const Eigen::Vector3d source_state(source_x, source_y, 0);
    const Eigen::Vector3d new_state(new_x, new_y, 0);
    const double norm_val = (new_state - source_state).norm();
    return (norm_val > 0.1) ? static_cast<int>(norm_val*100) : 10;
}

void NavLattice2D::add_successor(
    std::vector<int> *succ_ids,
    std::vector<int> *costs,
    const int& x,
    const int& y,
    const int& workspace,
    const int& new_x,
    const int& new_y,
    const int& new_workspace) {
    if (!is_valid_state(new_x, new_y, new_workspace)) {
        return;
    }

    const int cost = get_action_cost(x, y, workspace, new_x, new_y, new_workspace);

    State2D* new_state = get_state(new_x, new_y, new_workspace);
    if (new_state == NULL) {
        new_state = create_new_state(new_x, new_y, new_workspace);
    }
    const int state_id = get_state_id(new_x, new_y, new_workspace);

    succ_ids->push_back(state_id);
    costs->push_back(cost);
}

int NavLattice2D::set_start_state(
    const int& x,
    const int& y,
    const int& workspace) {
    ROS_INFO("[NavLattice2D] Start state: (%d, %d, %d)", x, y, workspace);
    if (!is_valid_state(x, y, workspace)) {
        ROS_ERROR("[NavLattice2D] Invalid Start State");
        return -1;
    }

    start_id_ = get_state_id(x, y, workspace);
    create_new_state(x, y, workspace);
    return start_id_;
}

int NavLattice2D::set_goal_state(
    const int& x,
    const int& y,
    const int& workspace) {
    ROS_INFO("[NavLattice2D] Goal state: (%d, %d, %d)", x, y, workspace);
    if (!is_valid_state(x, y, workspace)) {
        ROS_ERROR("[NavLattice2D] Invalid Goal State");
        return -1;
    }

    goal_id_ = get_state_id(x, y, workspace);
    create_new_state(x, y, workspace);
    return goal_id_;
}

void NavLattice2D::print_state(const int& state_id, const double& cost) {
    if (state_id >= state_ID_to_coord_map_.size()) {
        return;
    }
    const State2D* state = state_ID_to_coord_map_[state_id];
    const int x = state->x;
    const int y = state->y;
    const int workspace = state->workspace;

    ROS_INFO("State %d: (%d, %d, %d) -> %.3f", state_id, x, y, workspace, cost);
}

void NavLattice2D::get_succs(
    const int& source_state_id,
    std::vector<int> *succ_ids,
    std::vector<int> *costs) {
    // Goal state should be absorbing
    if (source_state_id == goal_id_) {
        return;
    }

    succ_ids->reserve(mprims_.size() + 1);
    costs->reserve(mprims_.size() + 1);

    if (source_state_id >= state_ID_to_coord_map_.size()) {
        ROS_ERROR("[NavLattice2D] Cannot expand node that doesn't exist.");
        return;
    }
    const State2D* state = state_ID_to_coord_map_[source_state_id];
    const int x = state->x;
    const int y = state->y;
    const int workspace = state->workspace;

    // Iterate through the motion primitives
    for (int idx = 0; idx < mprims_.size(); ++idx) {
        const int new_x = x + mprims_[idx].first;
        const int new_y = y + mprims_[idx].second;
        add_successor(succ_ids, costs, x, y, workspace, new_x, new_y, workspace);
    }

    // If there is a succesor on another workspace, add it to the succs list.
    const int connected_workspace = find_connected_workspace(x, y, workspace);
    if (connected_workspace != workspace) {
        add_successor(succ_ids, costs, x, y, workspace, x, y, connected_workspace);
    }
}

int NavLattice2D::find_or_create_signature(const std::vector<int>& signature) {
    int signature_id = get_signature_id(signature);
    if (signature_id < 0) {
        return create_new_signature(signature);
    }
    return signature_id;
}

int NavLattice2D::get_signature_id(const std::vector<int>& signature) {
    const auto map_elem = signature_to_ID_.find(signature);
    if (map_elem == signature_to_ID_.end()) {
        return -1;
    }
    return map_elem->second;
}

int NavLattice2D::create_new_signature(const std::vector<int>& signature) {
    const int signature_id = static_cast<int>(ID_to_signature_.size());
    signature_to_ID_[signature] = signature_id;
    ID_to_signature_.push_back(signature);
    return signature_id;
}

void NavLattice2D::get_signature(
    const int& signature_id,
    std::vector<int>* signature) {
    if (signature_id >= ID_to_signature_.size()) {
        ROS_ERROR("[NavLattice4D] Invalid signature ID.");
        return;
    }
    *signature = ID_to_signature_[signature_id];
}

}  // namespace graphs
}  // namespace footstep_planner
