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

#ifndef SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_PLANNERS_HBSP_H_
#define SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_PLANNERS_HBSP_H_

#include <footstep_planner/graphs/homotopy_information.h>
#include <footstep_planner/graphs/nav_lattice_2D.h>
#include <vector>
#include <set>
#include <unordered_set>

namespace footstep_planner {
namespace planners {

// An unordered set of allowed suffixes for signatures
typedef std::unordered_set<std::vector<int>> SuffixSet;

// A set of h-augmented goal vertices
typedef std::unordered_set<graphs::Vertex> GoalSet;

// Maps an h-augmented vertex to a cost
typedef std::unordered_map<graphs::Vertex, int> VertexCostMap;

class VertexCostMapComparator {
 public:
    explicit VertexCostMapComparator(const VertexCostMap& cost_map) : \
        cost_map_(cost_map) {}
        bool operator()(const graphs::Vertex& v1,
                        const graphs::Vertex& v2) const {
            return cost_map_.find(v1)->second <= cost_map_.find(v2)->second;
        }

 private:
    const VertexCostMap& cost_map_;
};

// This class implements the homotopy-based shortest path algorithm
// The algorithm is run on a 2D Navigation Lattice and uses the same state IDs
// stored in NavLattice2D
class HBSP {
 public:
    // homotopy_info The object used for augmented the state with the signature
    // graph The graph on which the planner runs
    // end_id The ID of the goal state (must be same goal ID from the graph)
    // start_id The ID of the start state (must be the same start ID from graph)
    // signatures The un-reduced signatures provided by the user
    // h_signatures The reduced signatures provided by the user
    HBSP(
        const std::shared_ptr<graphs::HomotopyInformation> homotopy_info,
        const std::shared_ptr<graphs::NavLattice2D> graph,
        const int &end_id,
        const int &start_id,
        const std::vector<std::vector<int>>& signatures,
        const std::vector<std::vector<int>>& h_signatures);

    // Given the index of the heuristic (hidx) and the state this function
    // returns the cost for the hidx'th heuristic.
    //
    // If the cost for the desired state has not been computed, HBSP is run on
    // demand. If the HBSP cost > anchor_heuristic_value, the search is
    // terminated. Note, anchor_heuristic_value is actually w2 * anchor value of
    // the state
    int get_cost(
        const int &hidx,
        const int &x,
        const int &y,
        const int &workspace,
        const int &anchor_heuristic_value,
        const std::vector<int>& signature);

    VertexCostMap get_hbsp_cost_map() const { return HBSP_cost_map_; }

    int get_hbsp_max_cost() const { return hbsp_max_cost_; }

    SuffixSet get_suffixes() const { return suffixes_; }

 private:
    // Creates a set of valid suffixes
    void create_suffix_set(
        const std::vector<std::vector<int>>& signatures,
        const std::vector<std::vector<int>>& h_signatures);

    // Gets the cost of the state with the desired signature
    int get_cost(
        const int &hidx,
        const int &state_id,
        const int desired_signature_id);

    // Runs the HBSP planner untils either all the goal states have been
    // expanded or (if the planner is being run online) terminates if the HBSP
    // cost > anchor_heuristic_value
    int update_cost_map(
        const bool offline_search,
        const int &end_id,
        const int &start_id,
        const std::vector<int>& goal_signature_ids,
        const int &anchor_heuristic_value = INFINITECOST);

    // This function gets the desired signature of the given state
    // More specifically, it concatenates the given signature with the
    // hidx'th signature the user provided
    //
    // Note we get the cost of the state (x, y, workspace, desired_signature)
    int get_desired_signature_id(
        const int &hidx,
        const std::vector<int>& signature);

    // The start and goal IDs
    int start_id_;
    int end_id_;

    // The user-provided workspaces
    std::vector<std::vector<int> > h_signatures_;

    // The set of valid suffixes
    SuffixSet suffixes_;

    // A mapping between the HBSP planner's vertices and their cost
    VertexCostMap HBSP_cost_map_;

    // The priority queue used by HBSP
    std::set<graphs::Vertex, VertexCostMapComparator> *Q_;

    // The max HBSP cost found (for visualization purposes)
    int hbsp_max_cost_;

    std::shared_ptr<graphs::HomotopyInformation> homotopy_info_;
    std::shared_ptr<graphs::NavLattice2D> graph_;
};

}  // namespace planners
}  // namespace footstep_planner

#endif  // SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_PLANNERS_HBSP_H_
