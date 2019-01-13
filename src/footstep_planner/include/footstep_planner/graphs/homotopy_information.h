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

#ifndef SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_GRAPHS_HOMOTOPY_INFORMATION_H_
#define SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_GRAPHS_HOMOTOPY_INFORMATION_H_

#include <footstep_planner/environment/environment_interpreter.h>
#include <footstep_planner/utils/helpers.h>
#include <map>
#include <set>
#include <utility>
#include <string>
#include <vector>

namespace footstep_planner {
namespace graphs {

typedef std::set<utils::Coordinates2D> Beams;

// This class is used to extract homotopy information from the given projected
// environments
class HomotopyInformation {
 public:
    HomotopyInformation(
        const std::shared_ptr<environment::proto::EnvironmentProjections>
            env_projections);

    // This function projects the discretized successor and parent state
    // (x, y, platform_z) -> (x, y, workspace_idx) and finds the signature of the
    // successor state.
    //
    // See src/proto/environment_projections/environment_projections.proto for
    // more information on mappings between various spaces
    void project_and_get_signature(
        const Eigen::Vector3i& successor,
        const Eigen::Vector3i& parent,
        const std::vector<int>& curr_signature,
        std::vector<int>* succ_signature);

    // Finds the signature for the successor state
    // More specifically it concatenates curr_signature with the signature of
    // of transitioning from the parent state to the successor state. This
    // concatenated state is stored in succ_signature.
    void get_signature(
        const int& succ_x,
        const int& succ_y,
        const int& succ_workspace,
        const int& curr_x,
        const int& curr_y,
        const int& curr_workspace,
        const std::vector<int>& curr_signature,
        std::vector<int>* succ_signature);

    // This function finds all the beams and gates in the given environment,
    // packs them into a protobuf and writes them to the given files in binary
    // form
    bool find_and_save_beams_and_gates(
        const std::string& beamfile,
        const std::string& gatefile);

    // Reads the protobufs from the given files and unpacks them
    bool read_beams_and_gates(
        const std::string& beamfile,
        const std::string& gatefile);

    // This function concatenates two signatures and reduce the concatenation if
    // necessary; concatenated_signature = h(signature_1 * signature_2)
    void concatenate_signatures(
        const std::vector<int>& signature_1,
        const std::vector<int>& signature_2,
        std::vector<int>* concatenated_signature);

    void print_beams();
    void print_gates();

 private:
    // Returns true if the letter is a gate and false otherwise
    bool is_gate(const int& letter) { return abs(letter) >= num_beams_; }

    // Returns true if the letter is a beam and false otherwise
    bool is_beam(const int& letter) { return abs(letter) < num_beams_; }

    // Given the letter for a beam, this function finds which workspace it's on
    int find_beam_workspace(const int& letter);

    // Finds the number of beams in the environment
    void find_num_beams();

    // Finds the beams and gates in the environment
    void find_beams_and_gates();

    // Finds the letters of the beams crossed when transitioning from
    // (x_low, y_low) to (x_high, y_high). If the beam crossed is b_k then the
    // letter stored in the beams_crossed vector is direction * k
    //
    // The direction of the letters is positive if
    // (x_low, y_low) < (x_high, y_high) and is negative otherwise.
    void get_beams_crossed(
        const int& x_high,
        const int& x_low,
        const int& y_high,
        const int& y_low,
        const int& workspace,
        const int& direction,
        std::vector<int>* beams_crossed);

    // Given (x, y, platform_z) returns the corresponding workspace index
    //
    // See src/proto/environment_projections/environment_projections.proto for
    // more information on mappings between various spaces
    int get_workspace_2d_idx(const Eigen::Vector3i& pos) const;

    // Returns the surface_z value given (x, y, workspace_idx)
    //
    // See src/proto/environment_projections/environment_projections.proto for
    // more information on mappings between various spaces
    int get_surface_z(
        const int x,
        const int y,
        const int workspace_idx) const;

    // Given (x, y, platform_z) returns the corresponding the
    // (lower_workspace_idx, upper_workspace_idx) pair
    //
    // See src/proto/environment_projections/environment_projections.proto for
    // more information on mappings between various spaces
    std::pair<int, int> get_workspace_indices(const Eigen::Vector3i& pos) const;

    // Helper function for finding all the cells in the projection that belongs
    // to an obstacle O_n
    void find_neighbor_obstacle_cells(
        const Projection& projection,
        std::vector<int>* visited,
        std::vector<utils::Coordinates2D>* neighbors);

    // Returns true if workspace_indices has a lower_workspace_idx and false
    // otherwise
    bool has_lower_workspace_idx(const std::pair<int, int>& workspace_indices) {
        return (workspace_indices.first > -1);
    }

    // Returns true if workspace_indices has an upper_workspace_idx and false
    // otherwise
    bool has_upper_workspace_idx(const std::pair<int, int>& workspace_indices) {
        return (workspace_indices.second > -1);
    }

    // Returns the lower workspace index
    int lower_workspace_idx(const std::pair<int, int>& workspace_indices) {
        return workspace_indices.first;
    }

    // Returns the upper workspace index
    int upper_workspace_idx(const std::pair<int, int>& workspace_indices) {
        return workspace_indices.second;
    }

    // The number of beams
    int num_beams_;

    // A vector of sets where each set contains the coordinates of point the
    // beam is extended from; every set of beams is associated with a
    // workspace/projection
    // e.g. beam_points[1] gives you a set of beams on the 1st workspace
    std::vector<Beams> beam_points_;

    // A set of gates where a gate is represented as a
    // (lower_workspace_idx, upper_workspace_idx) pair
    std::set<std::pair<int, int>> gates_;

    // This vector contains the number of beams (i-1)'th workspace and is used
    // to calculated the letter of the beam
    // e.g. num_prev_workspace_beams_[0] = 0, num_prev_workspace_beams_[1]
    // contains the number of beams on the 0'th workspace, etc.
    //
    // The letter of the beam is determined as follows: let i be the workspace
    // the beam b_k is on and let b_k be the j'th beam on that workspace then
    // the letter of beam b_k is num_prev_workspace_beams_[i] + j
    std::vector<int> num_prev_workspace_beams_;

    // The unpacked protobuf mappings
    std::vector<std::pair<int, int>> surface_3d_to_indices_;
    std::vector<int> workspace_3d_to_2d_;

    // The unpacked projections
    std::vector<Projection> projections_;

    // The number of rows and columns in each projection
    int num_rows_;
    int num_cols_;
};

}  // namespace graphs
}  // namespace footstep_planner

#endif  // SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_GRAPHS_HOMOTOPY_INFORMATION_H_

