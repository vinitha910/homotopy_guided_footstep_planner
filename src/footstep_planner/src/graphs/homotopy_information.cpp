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
#include <footstep_planner/proto/environment_projections/unpack_from_proto.h>
#include <footstep_planner/proto/homotopy_information/pack_into_proto.h>
#include <footstep_planner/proto/homotopy_information/unpack_from_proto.h>
#include <homotopy_information.pb.h>
#include <iostream>
#include <fstream>

namespace footstep_planner {
namespace graphs {

HomotopyInformation::HomotopyInformation(
    const std::shared_ptr<environment::proto::EnvironmentProjections>
    env_projections) {
    unpack_from_proto(&num_rows_, &num_cols_, &projections_, env_projections);
    unpack_from_proto(&surface_3d_to_indices_, env_projections);
    unpack_from_proto(&workspace_3d_to_2d_, env_projections);
}

void HomotopyInformation::print_beams() {
    if (beam_points_.empty()) {
        std::cout << "THERE ARE NO BEAMS" << std::endl;
        return;
    }

    int beam_idx = 0;
    for (const auto beams : beam_points_) {
        for (const auto beam : beams) {
            std::cout << "t_" << beam_idx << ": (" << beam.x
                  << ", " << beam.y << ")" << std::endl;
            ++beam_idx;
        }
    }
}

void HomotopyInformation::print_gates() {
    if (gates_.empty()) {
        std::cout << "THERE ARE NO GATES" << std::endl;
        return;
    }

    int gate_idx = num_beams_;
    for (const auto gate : gates_) {
        std::cout << "t_" << gate_idx << ": (workspace_" << gate.first
                  << ", workspace_" << gate.second << ")" << std::endl;
        ++gate_idx;
    }
}

void HomotopyInformation::find_num_beams() {
    num_beams_ = 0;
    num_prev_workspace_beams_ = std::vector<int>(beam_points_.size());
    for (int i = 0; i < beam_points_.size(); ++i) {
        num_prev_workspace_beams_[i] = num_beams_;
        num_beams_ += beam_points_[i].size();
    }
}

void HomotopyInformation::find_neighbor_obstacle_cells(
    const Projection& projection,
    std::vector<int>* visited,
    std::vector<utils::Coordinates2D>* neighbors) {
    const std::vector<int> dX = {1, -1, 0, 0};
    const std::vector<int> dY = {0, 0, 1, -1};

    while (!neighbors->empty()) {
        const utils::Coordinates2D neighbor = neighbors->front();
        for (int i = 0; i < dX.size(); ++i) {
            const int xn = neighbor.x + dX[i];
            const int yn = neighbor.y + dY[i];
            const int idx = yn * num_cols_ + xn;

            // If the index for the cell is valid, the cell has not been
            // visited and the cell is either and obstacle or workspace
            // boundary cell, add to neighbors list and mark as visited
            if (idx < visited->size() && !((*visited)[idx]) &&
               (projection.data[idx] == environment::OBSTACLE_CELL ||
                projection.data[idx] == environment::WORKSPACE_BOUNDARY_CELL)) {
                neighbors->push_back({.x = xn, .y = yn});
                (*visited)[idx] = 1;
            }
        }
        neighbors->erase(neighbors->begin());
    }
}

void HomotopyInformation::find_beams_and_gates() {
    std::vector<utils::Coordinates2D> neighbors;
    beam_points_ = std::vector<Beams>(projections_.size(), Beams());

    // Loop through each projected workspace
    for (int i = 0; i < projections_.size(); ++i) {
        std::vector<int> visited(num_rows_*num_cols_, 0);

        // Loop through each cell in the projected workspace
        for (int x = 0; x < num_cols_; ++x) {
            for (int y = 0; y < num_rows_; ++y) {
                const int workspace_2d_idx = (y * num_cols_) + x;
                const int surface_z =
                    projections_[i].workspace_2d_to_surface_3d[workspace_2d_idx];
                const int surface_3d_idx =
                    num_cols_ * ((num_rows_ * surface_z) + y) + x;

                if (surface_z == -1) {
                    continue;
                }

                // If the x, y, z is a gate, add it to the set of gates
                const auto workspace_indices =
                    surface_3d_to_indices_[surface_3d_idx];
                if (has_upper_workspace_idx(workspace_indices)) {
                    const int lower_workspace =
                        lower_workspace_idx(workspace_indices);
                    const int upper_workspace =
                        upper_workspace_idx(workspace_indices);
                    gates_.insert(
                        std::pair<int, int>(lower_workspace, upper_workspace));
                }

                // Skip visited cells, otherwise check if cell has not been
                // visited and is an obstacle cell
                if (visited[workspace_2d_idx] == 1) {
                    continue;
                } else if (projections_[i].data[workspace_2d_idx] == \
                    environment::OBSTACLE_CELL ||
                    projections_[i].data[workspace_2d_idx] == \
                        environment::WORKSPACE_BOUNDARY_CELL) {
                    // Make the first found cell for obstacle O_n the point from
                    // which beam b_n will be extended in the +y direction
                    utils::Coordinates2D obs_coors = {.x = x, .y = y};
                    const int init_size = beam_points_[i].size();
                    beam_points_[i].insert(obs_coors);
                    neighbors.push_back({.x = x, .y = y});

                    // If the new beam is parallel to an existing beam, find
                    // another beam from obstable O_n
                    if (beam_points_[i].size() == init_size) {
                        break;
                    }

                    // Find all the neighboring obstacle cells for obstacle O_n
                    // and mark them as visited
                    find_neighbor_obstacle_cells(
                        projections_[i],
                        &visited,
                        &neighbors);
                }

                // Mark the cell as visited
                visited[workspace_2d_idx] = 1;
            }
        }
    }
    find_num_beams();
}

bool HomotopyInformation::find_and_save_beams_and_gates(
    const std::string& beamfile,
    const std::string& gatefile) {
    find_beams_and_gates();

    // Build the beam protobuf messages and write them to disk
    std::vector<proto::Beams_Beam> beams;
    for (int workspace = 0; workspace < beam_points_.size(); workspace++) {
        for (auto beam = beam_points_[workspace].begin();
                beam != beam_points_[workspace].end(); ++beam) {
            proto::Beams_Beam beam_proto;
            beams.push_back(
                proto::pack_into_proto(
                    beam->x,
                    beam->y,
                    workspace,
                    std::distance(beam_points_[workspace].begin(), beam) + \
                    num_prev_workspace_beams_[workspace]));
        }
    }
    proto::Beams beams_proto;
    proto::pack_into_proto(&beams_proto, beams);

    std::fstream output_beam(beamfile.c_str(),
        std::ios::out | std::ios::trunc | std::ios::binary);
    if (!beams_proto.SerializeToOstream(&output_beam)) {
        ROS_ERROR("[HomotopyInformation] Failed to serialize Beams protobuf");
        return false;
    }

    // Build the gate protobuf messages and write them to disk
    std::vector<proto::Gates_Gate> gates;
    for (auto gate = gates_.begin(); gate != gates_.end(); ++gate) {
        gates.push_back(
            proto::pack_into_proto(
                gate->first,
                gate->second,
                std::distance(gates_.begin(), gate) + num_beams_));
    }
    proto::Gates gates_proto;
    proto::pack_into_proto(&gates_proto, gates);

    std::fstream output_gate(gatefile.c_str(),
        std::ios::out | std::ios::trunc | std::ios::binary);
    if (!gates_proto.SerializeToOstream(&output_gate)) {
        ROS_ERROR("[HomotopyInformation] Failed to serialize Gates protobuf");
        return false;
    }

    return true;
}

bool HomotopyInformation::read_beams_and_gates(
    const std::string& beamfile,
    const std::string& gatefile) {
    // Read the protobuf message
    std::fstream beams_input(beamfile, std::ios::in |
        std::ios::binary);
    proto::Beams beams;
    if (!beams.ParseFromIstream(&beams_input)) {
        ROS_ERROR("Error: failed to parse Beams protbuf");
        return false;
    }
    proto::unpack_from_proto(&beam_points_, beams);

    std::fstream gates_input(gatefile, std::ios::in |
        std::ios::binary);
    proto::Gates gates;
    if (!gates.ParseFromIstream(&gates_input)) {
        ROS_ERROR("Error: failed to parse ValidSteppingCells protbuf");
        return false;
    }
    proto::unpack_from_proto(&gates_, gates);

    find_num_beams();

    return true;
}

int HomotopyInformation::get_workspace_2d_idx(
    const Eigen::Vector3i& pos) const {
    const int index =
        (pos.z() * num_rows_ * num_cols_) + (pos.y() * num_cols_) + pos.x();

    // Convert (x, y, z) -> (x, y, workspace)
    if (index >= workspace_3d_to_2d_.size()) {
        ROS_ERROR(
            "[HomotopyInformation] (%d, %d, %d) is not a valid platform cell",
            pos.x(), pos.y(), pos.z());
        return -1;
    }
    return workspace_3d_to_2d_[index];
}

int HomotopyInformation::get_surface_z(
    const int x,
    const int y,
    const int workspace_idx) const {
    // Convert (x, y, workspace_idx)-> (x, y, surface_z)
    const int index = (y * num_cols_) + x;
    if (index >= projections_[workspace_idx].workspace_2d_to_surface_3d.size()) {
        ROS_ERROR(
            "[HomotopyInformation] (%d, %d) is not a valid cell on surface %d",
            x, y, workspace_idx);
        return -1;
    }

    const int surface_z =
        projections_[workspace_idx].workspace_2d_to_surface_3d[index];
    if (surface_z == -1) {
        ROS_ERROR("[HomotopyInformation] There is no surface z-value at x: %d, "
                  "y: %d, workspace: %d", x, y, workspace_idx);
    }

    return surface_z;
}

std::pair<int, int> HomotopyInformation::get_workspace_indices(
    const Eigen::Vector3i& pos) const {
    const int workspace_index = get_workspace_2d_idx(pos);
    const int surface_z =
        get_surface_z(pos.x(), pos.y(), workspace_index);

    // Convert (x, y, surface_z) -> workspace_indices
    const int index =
        (surface_z * num_rows_ * num_cols_) + (pos.y() * num_cols_) + pos.x();

    if (index >= surface_3d_to_indices_.size()) {
        ROS_ERROR(
            "[HomotopyInformation] (%d, %d) is not a valid cell on workspace %d",
            pos.x(), pos.y(), workspace_index);
        return std::make_pair(-1, -1);
    }

    return surface_3d_to_indices_[index];
}

void HomotopyInformation::project_and_get_signature(
    const Eigen::Vector3i& successor,
    const Eigen::Vector3i& parent,
    const std::vector<int>& curr_signature,
    std::vector<int>* succ_signature) {
    const auto curr_workspace_indices = get_workspace_indices(parent);
    const auto succ_workspace_indices = get_workspace_indices(successor);

    int curr_workspace;
    int succ_workspace;

    // CASE 1:  Going from one gate to another
    // CASE 2: Going from one workspace point to another
    // CASE 3: Going from a workspace point to a gate
    // CASE 4: Going from a gate to a workspace point; This is the only case in
    // which we are considered to cross a gate
    if (has_upper_workspace_idx(curr_workspace_indices) &&
        has_upper_workspace_idx(succ_workspace_indices)) {
        curr_workspace = lower_workspace_idx(curr_workspace_indices);
        succ_workspace = curr_workspace;
    } else if ((!has_upper_workspace_idx(curr_workspace_indices)) &&
               (!has_upper_workspace_idx(succ_workspace_indices))) {
        curr_workspace = lower_workspace_idx(curr_workspace_indices);
        succ_workspace = lower_workspace_idx(succ_workspace_indices);
    } else if (!has_upper_workspace_idx(curr_workspace_indices) &&
               has_upper_workspace_idx(succ_workspace_indices)) {
        curr_workspace = lower_workspace_idx(curr_workspace_indices);
        succ_workspace = curr_workspace;
    } else if (has_upper_workspace_idx(curr_workspace_indices) &&
               !has_upper_workspace_idx(succ_workspace_indices)) {
        succ_workspace = lower_workspace_idx(succ_workspace_indices);
        curr_workspace =
            (lower_workspace_idx(curr_workspace_indices) == succ_workspace) ?
                upper_workspace_idx(curr_workspace_indices) : \
                lower_workspace_idx(curr_workspace_indices);
    } else {
        ROS_ERROR("[HomotopyInformation] Failed to project platform state onto"
                  " workspaces");
    }
    get_signature(successor.x(), successor.y(), succ_workspace, parent.x(),
                  parent.y(), curr_workspace, curr_signature, succ_signature);
}

void HomotopyInformation::get_beams_crossed(
    const int& x_high,
    const int& x_low,
    const int& y_high,
    const int& y_low,
    const int& workspace,
    const int& direction,
    std::vector<int>* beams_crossed) {
    const auto beams = beam_points_[workspace];

    // Takes beam that is at x_low or after
    Beams::iterator iter_low =
        beams.lower_bound({.x = x_low, .y = y_low});
    Beams::iterator iter_high =
        beams.upper_bound({.x = x_high, .y = y_high});

    // Find all the beams that have been crossed
    for (auto iter = iter_low; iter != iter_high; ++iter) {
        const auto beam_coor = *iter;
        if (x_high > beam_coor.x && y_high > beam_coor.y &&
            x_low <= beam_coor.x && y_low > beam_coor.y) {
            const int letter_offset = num_prev_workspace_beams_[workspace];
            const int letter =
                direction * (letter_offset + \
                    std::distance(beams.begin(), iter));
            beams_crossed->push_back(letter);
        }
    }
}

void HomotopyInformation::get_signature(
    const int& succ_x,
    const int& succ_y,
    const int& succ_workspace,
    const int& curr_x,
    const int& curr_y,
    const int& curr_workspace,
    const std::vector<int>& curr_signature,
    std::vector<int>* succ_signature) {
    std::vector<int> beams_crossed;
    succ_signature->assign(curr_signature.begin(), curr_signature.end());

    // Collect letters from left to right when curr_x < succ_x
    // Collect letters from right to left when curr_x > succ_x
    // If curr_x == succ_x then no beams have been crossed
    // If curr_workspace == succ_workspace then no gates have been crossed
    if (curr_x < succ_x) {
        get_beams_crossed(
            succ_x,
            curr_x,
            succ_y,
            curr_y,
            curr_workspace,
            1,
            &beams_crossed);
        if (curr_workspace != succ_workspace && beams_crossed.empty()) {
            get_beams_crossed(
                succ_x,
                curr_x,
                succ_y,
                curr_y,
                succ_workspace,
                1,
                &beams_crossed);
        }
    } else if (curr_x > succ_x) {
        get_beams_crossed(
            curr_x,
            succ_x,
            curr_y,
            succ_y,
            curr_workspace,
            -1,
            &beams_crossed);
        if (curr_workspace != succ_workspace && beams_crossed.empty()) {
            get_beams_crossed(
                curr_x,
                succ_x,
                curr_y,
                succ_y,
                succ_workspace,
                -1,
                &beams_crossed);
        }
    } else if (curr_x == succ_x && curr_workspace == succ_workspace) {
        return;
    }

    // Find the gate that has been crossed
    if (curr_workspace != succ_workspace) {
        auto gate_idx = gates_.find(std::make_pair(curr_workspace, succ_workspace));
        if (gate_idx == gates_.end()) {
            gate_idx = gates_.find(std::make_pair(succ_workspace, curr_workspace));
        }

        if (gate_idx == gates_.end()) {
            ROS_ERROR("There is no gate connecting points (%d, %d, %d) and "
                      "(%d, %d, %d)", curr_x, curr_y, curr_workspace, succ_x,
                      succ_y, succ_workspace);
            return;
        }

        const int direction = (succ_workspace > curr_workspace) ? 1 : -1;
        const int letter =
            direction * \
                (std::distance(gates_.begin(), gate_idx) + num_beams_);

        // If both a gate and a beam(s) have been crossed
        if (!beams_crossed.empty()) {
            // Find the workspace the beam is on
            const int beam_workspace =
                find_beam_workspace(beams_crossed.front());

            beams_crossed.push_back(letter);

            // If the beam is on the successor workspace, flip the gate and
            // beam
            if (beam_workspace == succ_workspace) {
                std::iter_swap(
                    std::prev(beams_crossed.end(), 2),
                    std::prev(beams_crossed.end(), 1));
            }
        } else {  // If only a gate has been crossed
            beams_crossed.push_back(letter);
        }
    }

    // Update the successor signature
    for (auto it = beams_crossed.rbegin(); it != beams_crossed.rend(); ++it) {
        const int letter = *it;
        if (!succ_signature->empty() && succ_signature->back() == -1 * letter) {
            succ_signature->pop_back();
        } else {
            succ_signature->push_back(letter);
        }
    }
}

int HomotopyInformation::find_beam_workspace(const int& letter) {
    // Find the workspace the beam is on
    auto workspace_iterator = std::upper_bound(
        num_prev_workspace_beams_.begin(),
        num_prev_workspace_beams_.end(),
        abs(letter));

    if (workspace_iterator == num_prev_workspace_beams_.end() &&
        abs(letter) >= num_beams_) {
        ROS_ERROR("[HomotopyInformation] Invalid beam letter %d.", letter);
        return -1;
    } else if (workspace_iterator == num_prev_workspace_beams_.end() &&
               abs(letter) < num_beams_) {
        workspace_iterator = std::prev(num_prev_workspace_beams_.end(), 1);
    }

    return std::distance(
        num_prev_workspace_beams_.begin(),
        std::prev(workspace_iterator, 1));
}

void HomotopyInformation::concatenate_signatures(
    const std::vector<int>& signature_1,
    const std::vector<int>& signature_2,
    std::vector<int>* concatenated_signature) {
    concatenated_signature->assign(signature_1.begin(), signature_1.end());
    std::vector<int> added_signature = signature_2;
    bool reduce = true;
    while (reduce &&
          !concatenated_signature->empty() &&
          !added_signature.empty()) {
        // If the last letter of the first signature is a gate
        if (is_gate(concatenated_signature->back()) && \
            is_beam(added_signature.front())) {
            // Find the gate given the letter of the gate
            const int gate_idx =
                abs(concatenated_signature->back()) - num_beams_;
            const auto gate = std::next(gates_.begin(), gate_idx);
            if (gate == gates_.end()) {
                ROS_ERROR("[HomotopyInformation] Invalid gate letter");
                return;
            }

            const int beam_workspace = find_beam_workspace(added_signature.front());

            // If the direction of the letter is positive (i.e. we transitioned
            // from the lower workspace to the upper workspace) the parent's
            // workspace is the lower workspace and the successor's worksapce is
            // the upper workspace.
            const int curr_workspace =
                (concatenated_signature->back() > 0) ? \
                    (*gate).first : (*gate).second;
            const int succ_workspace =
                (concatenated_signature->back() > 0) ? \
                    (*gate).second : (*gate).first;

            // If added_signature's first two letters are a beam and gate
            // respectively and the second letter in added_signature ==
            // the opposite of last letter in the concatenated signature
            if (added_signature.size() > 1 &&
                is_beam(added_signature.front()) &&
                is_gate(*(added_signature.begin() + 1)) &&
                concatenated_signature->back() == \
                    -1*(*(added_signature.begin() + 1))) {
                // If both the beams and gate can be reduced reduce them
                if (*(concatenated_signature->end() - 2) == \
                        -1*added_signature.front()) {
                    concatenated_signature->erase(
                        concatenated_signature->end() - 2,
                        concatenated_signature->end());
                    added_signature.erase(
                        added_signature.begin(),
                        added_signature.begin() + 2);

                // If the beams is on the gate's parent workspace, reduce the
                // gate and add the beam
                } else if (beam_workspace == curr_workspace) {
                    concatenated_signature->pop_back();
                    added_signature.pop_back();
                    concatenated_signature->push_back(added_signature.front());
                    added_signature.erase(added_signature.begin());

                // If the beam is on the gate's successor workspace, add
                // signature_2's beam and gate to the concatenated signature
                } else if (beam_workspace == succ_workspace) {
                    concatenated_signature->insert(
                        concatenated_signature->end(),
                        added_signature.begin(),
                        added_signature.begin() + 2);
                    added_signature.erase(
                        added_signature.begin(),
                        added_signature.begin() + 2);
                } else {
                    ROS_ERROR(
                        "[HomotopyInformation] The beam (%d) workspace %d is "
                        "not the gate's parent workspace %d or successor "
                        "workspace %d",
                        added_signature.front(),
                        beam_workspace,
                        curr_workspace,
                        succ_workspace);
                    reduce = false;
                }

            // If added_signature's first letter is a beam and the second letter
            // is not a gate
            } else if (added_signature.size() == 1 && \
                     is_beam(added_signature.front())) {
                // If beam is on gate's parent workspace, then place beam before
                // the gate in the concatenated signature
                if (beam_workspace == curr_workspace) {
                    concatenated_signature->push_back(added_signature.front());
                    // swap the last two elements
                    std::iter_swap(
                        concatenated_signature->end() - 1,
                        concatenated_signature->end() - 2);
                    added_signature.erase(added_signature.begin());

                // If beam is on gate's successor workspace idx, then place beam
                // after the gate
                } else if (beam_workspace == succ_workspace) {
                    concatenated_signature->push_back(added_signature.front());
                    added_signature.erase(added_signature.begin());
                } else {
                    ROS_ERROR(
                        "[HomotopyInformation] The beam (%d) workspace %d is "
                        "not the gate's parent workspace %d or successor "
                        "workspace %d",
                        added_signature.front(),
                        beam_workspace,
                        curr_workspace,
                        succ_workspace);
                    reduce = false;
                }
            } else {
                reduce = false;
            }
        } else if (
            concatenated_signature->back() == -1 * added_signature.front()) {
            concatenated_signature->pop_back();
            added_signature.erase(added_signature.begin());
        } else {
            reduce = false;
        }
    }

    if (!added_signature.empty()) {
        concatenated_signature->insert(
            concatenated_signature->end(),
            added_signature.begin(),
            added_signature.end());
    }
}

}  // namespace graphs
}  // namespace footstep_planner
