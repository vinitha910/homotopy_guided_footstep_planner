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

#include <footstep_planner/proto/homotopy_information/unpack_from_proto.h>

namespace footstep_planner {
namespace graphs {
namespace proto {

utils::Coordinates2D unpack_from_proto(const Beams_Beam& beam) {
    utils::Coordinates2D beam_coor;
    beam_coor.x = beam.x();
    beam_coor.y = beam.y();

    return beam_coor;
}

void unpack_from_proto(
    std::vector<std::set<utils::Coordinates2D>>* beams,
    const Beams& beams_proto) {
    for (int idx = 0; idx < beams_proto.beams_size(); ++idx) {
        const Beams_Beam beam = beams_proto.beams(idx);

        // Resize to fit the number of workspaces
        if (beam.workspace() + 1 > beams->size()) {
            beams->resize(beam.workspace() + 1, std::set<utils::Coordinates2D>());
        }

        (*beams)[beam.workspace()].insert(unpack_from_proto(beam));
    }
}

std::pair<int, int> unpack_from_proto(const Gates_Gate& gate) {
    return std::make_pair(
        gate.workspaces().lower_workspace_idx(),
        gate.workspaces().upper_workspace_idx());
}

void unpack_from_proto(
    std::set<std::pair<int, int>>* gates,
    const Gates& gates_proto) {
    for (int idx = 0; idx < gates_proto.gates_size(); ++idx) {
        const Gates_Gate gate = gates_proto.gates(idx);
        (*gates).insert(unpack_from_proto(gate));
    }
}

}  // namespace proto
}  // namespace graphs
}  // namespace footstep_planner
