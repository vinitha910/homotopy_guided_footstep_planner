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

#include <footstep_planner/planners/dijkstra.h>
#include <iostream>

namespace footstep_planner {
namespace planners {

Dijkstra::Dijkstra(
    const std::shared_ptr<graphs::NavLattice2D> graph,
    const int &start_id) : graph_(graph), start_id_(start_id) {
    update_cost_map();
}

void Dijkstra::update_cost_map() {
    // Create priority queue
    CostMapComparator cmp(dijkstra_cost_map_);
    std::set<int, CostMapComparator> Q = std::set<int, CostMapComparator>(cmp);

    // Insert first node into priority queue
    dijkstra_cost_map_[start_id_] = 0;
    Q.insert(start_id_);

    dijkstra_max_cost_ = 0;

    std::vector<int> succ_ids;
    std::vector<int> costs;

    while (!Q.empty()) {
        const int u = *(Q.begin());
        Q.erase(Q.begin());

        succ_ids.clear();
        costs.clear();
        graph_->get_succs(u, &succ_ids, &costs);
        assert(succ_ids.size() == costs.size());
        const int cost_u = dijkstra_cost_map_.at(u);

        for (int sidx = 0; sidx < succ_ids.size(); ++sidx) {
            const int alt = cost_u + costs[sidx];
            const int curr_vertex = succ_ids[sidx];

            auto dist_curr = dijkstra_cost_map_.find(curr_vertex);

            // If current vertex wasn't explored (i.e. if cost was not recorded)
            // or the new cost is less than cost currently recorded, add/
            // replace this vertice's g-value in the cost map
            if (dist_curr == dijkstra_cost_map_.end() ||
                alt < dist_curr->second) {
                dijkstra_cost_map_[curr_vertex] = alt;

                // Record the max cost for visualization purposed
                if (alt > dijkstra_max_cost_) {
                    dijkstra_max_cost_ = alt;
                }

                auto it = Q.find(curr_vertex);
                if (it != Q.end()) {
                    Q.erase(it);
                }
                Q.insert(curr_vertex);
            }
        }
    }
}

int Dijkstra::get_cost(const int& x, const int& y, const int& workspace) {
    const int connected_workspace =
        graph_->find_connected_workspace(x, y, workspace);

    if (connected_workspace != workspace) {
        if (!graph_->is_valid_state(x, y, connected_workspace)) {
            return INFINITECOST;
        }
        const int state_id = graph_->get_state_id(x, y, connected_workspace);
        if (dijkstra_cost_map_.count(state_id) > 0) {
            return dijkstra_cost_map_.at(state_id);
        }
    }

    if (!graph_->is_valid_state(x, y, workspace)) {
        return INFINITECOST;
    }
    const int state_id = graph_->get_state_id(x, y, workspace);
    if (dijkstra_cost_map_.count(state_id) > 0) {
        return dijkstra_cost_map_.at(state_id);
    }

    return INFINITECOST;
}

}  // namespace planners
}  // namespace footstep_planner
