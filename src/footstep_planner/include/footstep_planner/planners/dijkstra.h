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

#ifndef SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_PLANNERS_DIJKSTRA_H_
#define SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_PLANNERS_DIJKSTRA_H_

#include <footstep_planner/graphs/nav_lattice_2D.h>

namespace footstep_planner {
namespace planners {

typedef std::unordered_map<int, int> CostMap;

class CostMapComparator {
 public:
    explicit CostMapComparator(const CostMap& cost_map): cost_map_(cost_map) {}

    bool operator()(const int& v1,
                    const int& v2) const {
        return cost_map_.find(v1)->second <= cost_map_.find(v2)->second;
    }

 private:
    const CostMap& cost_map_;
};

// This class implements Dijkstra's algorithm on a 2D navigation lattice
class Dijkstra {
 public:
    Dijkstra(
        const std::shared_ptr<graphs::NavLattice2D> graph,
        const int &start_id);

    // The cost of the given state
    int get_cost(const int& x, const int& y, const int& workspace);

    int get_dijkstra_max_cost() const {return dijkstra_max_cost_; }

    CostMap get_dijkstra_cost_map() const { return dijkstra_cost_map_; }

 private:
    // Runs the Dijkstra's planner to completion
    void update_cost_map();

    CostMap dijkstra_cost_map_;

    std::shared_ptr<graphs::NavLattice2D> graph_;

    int start_id_;

    int dijkstra_max_cost_;
};

}  // namespace planners
}  // namespace footstep_planner

#endif  // SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_PLANNERS_DIJKSTRA_H_
