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

#include <footstep_planner/planners/hbsp.h>
#include <iostream>
#include <memory>
#include <cassert>

namespace footstep_planner {
namespace planners {

HBSP::HBSP(
    const std::shared_ptr<graphs::HomotopyInformation> homotopy_info,
    const std::shared_ptr<graphs::NavLattice2D> graph,
    const int &end_id,
    const int &start_id,
    const std::vector<std::vector<int>>& signatures,
    const std::vector<std::vector<int>>& h_signatures) {
    homotopy_info_ = homotopy_info;
    graph_ = graph;
    start_id_ = start_id;
    end_id_ = end_id;
    h_signatures_ = h_signatures;

    // Create a list of allowed suffixes using the un-reduced signatures
    create_suffix_set(signatures, h_signatures);

    // Initializing priority queue
    Q_ = new std::set<graphs::Vertex, VertexCostMapComparator>(
        VertexCostMapComparator(HBSP_cost_map_));

    // Create vector of goal signature IDs
    std::vector<int> goal_signature_ids(h_signatures_.size());
    for (const auto signature : h_signatures_) {
        const int signature_id = graph_->find_or_create_signature(signature);
        goal_signature_ids.push_back(signature_id);
    }
    update_cost_map(true, end_id_, start_id_, goal_signature_ids);
}

void HBSP::create_suffix_set(
    const std::vector<std::vector<int>>& signatures,
    const std::vector<std::vector<int>>& h_signatures) {
    suffixes_.clear();
    if (signatures != h_signatures) {
        // Loop through every un-reduced signature
        for (int i = 0; i < signatures.size(); ++i) {
            std::vector<int> signature = signatures[i];

            // Loop through every letter in signature
            for (auto letter = signature.begin();
                letter != signature.end();
                ++letter) {
                // Delete adjacent letters until a valid adjacent letter is
                // found
                bool valid_adjacent_letter = false;
                while (!valid_adjacent_letter) {
                    const auto adjacent_letter = std::next(letter, 1);
                    if (adjacent_letter == signature.end()) {
                        valid_adjacent_letter = true;
                    } else if (*adjacent_letter == *letter ||
                        *adjacent_letter == -1*(*letter)) {
                        signature.erase(adjacent_letter);
                    } else {
                        valid_adjacent_letter = true;
                    }
                }
            }

            // Add valid suffixes to suffixes set
            suffixes_.insert(std::vector<int>());
            for (int j = 1; j <= signature.size(); ++j) {
                std::vector<int> suffix(
                    signature.begin(),
                    std::next(signature.begin(),
                    j));
                suffixes_.insert(suffix);
            }
        }
    }

    // Add valid suffixes from the h-signature if not in the suffixes set
    for (int i = 0; i < h_signatures.size(); ++i) {
        const std::vector<int> h_signature = h_signatures[i];
        for (int j = 1; j <= h_signature.size(); ++j) {
            std::vector<int> suffix(
                h_signature.begin(),
                std::next(h_signature.begin(),
                j));
            suffixes_.insert(suffix);
        }
    }
}

int HBSP::update_cost_map(
    const bool offline_search,
    const int &end_id,
    const int &start_id,
    const std::vector<int>& goal_signature_ids,
    const int &anchor_heuristic_value) {
    // If the queue is empty, insert a vertex at the start_id with a cost of 0
    if (Q_->empty()) {
        const int signature_id =
            graph_->find_or_create_signature(std::vector<int>());
        graphs::Vertex init_v = {.id = start_id, .signature_id = signature_id};
        HBSP_cost_map_[init_v] = 0;
        hbsp_max_cost_ = 0;
        Q_->insert(init_v);
    }

    // Initializing goal set
    GoalSet goals;
    for (int i = 0; i < goal_signature_ids.size(); ++i) {
        goals.insert({.id = end_id, .signature_id = goal_signature_ids[i]});
    }

    std::vector<int> succ_ids;
    std::vector<int> costs;
    std::vector<int> succ_sig;

    while (!Q_->empty()) {
        const graphs::Vertex u = *(Q_->begin());
        const int cost_u = HBSP_cost_map_.at(u);

        if (goals.count(u) > 0) {
            goals.erase(u);
            if (goals.empty()) {
                ROS_INFO("[HBSP] All goals found.");
                return cost_u;
            }
        }

        // Stop running online HBSP when the cost of the node being expanded
        // is > w2 * anchor_heuristic_value for the goal state
        if (!offline_search && cost_u > anchor_heuristic_value) {
            ROS_DEBUG("[HBSP] Terminating online search.");
            return cost_u;
        }

        if (!offline_search) {
            ROS_DEBUG("%d < %d", cost_u, anchor_heuristic_value);
        }

        Q_->erase(Q_->begin());
        succ_ids.clear();
        costs.clear();
        succ_sig.clear();
        graph_->get_succs(u.id, &succ_ids, &costs);
        assert(succ_ids.size() == costs.size());

        for (int sidx = 0; sidx < succ_ids.size(); ++sidx) {
            succ_sig.clear();
            int parent_x, parent_y, parent_z, succ_x, succ_y, succ_z;
            graph_->get_coord_from_state_id(
                u.id, &parent_x, &parent_y, &parent_z);
            graph_->get_coord_from_state_id(
                succ_ids[sidx], &succ_x, &succ_y, &succ_z);

            std::vector<int> parent_signature;
            graph_->get_signature(u.signature_id, &parent_signature);
            homotopy_info_->get_signature(
                succ_x,
                succ_y,
                succ_z,
                parent_x,
                parent_y,
                parent_z,
                parent_signature, &succ_sig);

            const int succ_sig_id = graph_->find_or_create_signature(succ_sig);

            if (offline_search &&
                !succ_sig.empty() &&
                suffixes_.count(succ_sig) == 0) {
                succ_sig.clear();
                continue;

            // If HBSP is run on demand, do not allow signature to
            // loop around obstacles more than once
            } else if (!offline_search && !succ_sig.empty()) {
                std::vector<int> succ_sig_sorted(succ_sig);
                std::sort(succ_sig_sorted.begin(), succ_sig_sorted.end());
                const auto adjacent_letters =
                    std::adjacent_find(
                        succ_sig_sorted.begin(),
                        succ_sig_sorted.end());
                if (adjacent_letters != succ_sig_sorted.end()) {
                    succ_sig.clear();
                    continue;
                }
            }

            // Find current distance
            const int alt = cost_u + costs[sidx];
            const graphs::Vertex curr_vertex = {
                .id = succ_ids[sidx],
                .signature_id = succ_sig_id};

            // Check if current vertex was explored, (i.e. if cost is specified)
            const auto cost_curr = HBSP_cost_map_.find(curr_vertex);
            if (cost_curr == HBSP_cost_map_.end() || alt < cost_curr->second) {
                HBSP_cost_map_[curr_vertex] = alt;

                // Store max cost for visualizations
                if (offline_search && alt > hbsp_max_cost_) {
                    hbsp_max_cost_ = alt;
                }
                auto it = Q_->find(curr_vertex);
                if (it != Q_->end()) {
                    Q_->erase(it);
                }
                Q_->insert(curr_vertex);
            }
        }
    }

    // Set all unreachable goals to dist of infinity
    ROS_INFO("[HBSP] Some goals are unreachable.");
    for (const auto &g : goals) {
        HBSP_cost_map_.insert(std::make_pair(g, INFINITECOST));
    }

    return INFINITECOST;
}

int HBSP::get_desired_signature_id(
    const int &hidx,
    const std::vector<int>& signature) {
    const std::vector<int> desired_sig = h_signatures_.at(hidx - 1);
    std::vector<int> concatenated_signature;
    homotopy_info_->concatenate_signatures(
        desired_sig,
        signature,
        &concatenated_signature);

    return graph_->find_or_create_signature(concatenated_signature);
}

int HBSP::get_cost(
    const int &hidx,
    const int &state_id,
    const int desired_signature_id) {
    // The start state in the 2D graph is the goal state in the 4D graph
    if (graph_->is_start_state(state_id)) {
        return 0;
    }

    const graphs::Vertex v_inverse = {
        .id = state_id,
        .signature_id = desired_signature_id};

    if (HBSP_cost_map_.count(v_inverse) > 0) {
        return HBSP_cost_map_.at(v_inverse);
    }

    return INFINITECOST;
}

int HBSP::get_cost(
    const int &hidx,
    const int &x,
    const int &y,
    const int &workspace,
    const int &anchor_heuristic_value,
    const std::vector<int>& signature) {
    const int connected_workspace = graph_->find_connected_workspace(x, y, workspace);
    const int desired_signature_id = get_desired_signature_id(hidx, signature);

    // If there is an upper workspace index (i.e. the point is on a gate) then
    // return the cost of (x, y, upper_workspace) if it is not infinity
    if (workspace != connected_workspace) {
        // If (x, y, connected_workspace) is invalid return infinite cost
        if (!graph_->is_valid_state(x, y, connected_workspace)) {
            return INFINITECOST;
        }

        // Return the cost if it's not infinity
        const int state_id = graph_->get_state_id(x, y, connected_workspace);
        const int cost = get_cost(hidx, state_id, desired_signature_id);
        if (cost != INFINITECOST) {
            return cost;
        }
    }

    // If (x, y, workspace) is invalid return infinite cost
    if (!graph_->is_valid_state(x, y, workspace)) {
        return INFINITECOST;
    }

    // If the cost of (x, y, upper_workspace) is infinity or the point is not on
    // a gate, find the cost of (x, y, lower_workspace)
    //
    // If the cost is inifinity, then run HBSP on demand otherwise return cost
    const int state_id = graph_->get_state_id(x, y, workspace);
    const int cost = get_cost(hidx, state_id, desired_signature_id);
    if (cost == INFINITECOST) {
        if (Q_->empty()) {
            return INFINITECOST;
        }
        ROS_DEBUG(
            "Calling HBSP online for (%d, %d, %d) in search %d",
            x, y, workspace, hidx);
        std::vector<int> desired_signature;
        graph_->get_signature(desired_signature_id, &desired_signature);
        return update_cost_map(
            false,
            state_id,
            start_id_,
            {desired_signature_id},
            anchor_heuristic_value);
    }
    return cost;
}

}  // namespace planners
}  // namespace footstep_planner
