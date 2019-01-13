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

#ifndef SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_PROTO_ENVIRONMENT_PROJECTIONS_PACK_INTO_PROTO_H_
#define SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_PROTO_ENVIRONMENT_PROJECTIONS_PACK_INTO_PROTO_H_

#include <environment_projections.pb.h>
#include <utility>
#include <vector>

namespace footstep_planner {
namespace environment {
namespace proto {

void pack_into_proto(
    const int& lower_workspace_idx,
    const int& upper_workspace_idx,
    WorkspaceIndices* surface_indices);

// Populate a Valid Stepping cells proto
void pack_into_proto(
    const std::vector<bool>& stepping_cells,
    ValidSteppingCells* valid_stepping_cells_proto);

void pack_into_proto(
    const std::vector<std::vector<proto::EnvironmentProjection_CellType>>& data,
    const std::vector<std::vector<int>>& workspace_2d_to_surface_3d,
    std::vector<proto::EnvironmentProjection>* projection);

void pack_into_proto(
    const int& rows,
    const int& cols,
    const std::vector<EnvironmentProjection>& projections,
    const std::vector<std::pair<int, int>>& surface_indices,
    const std::vector<int>& workspace_3d_to_2d,
    const std::vector<int>& surface_3d_to_stepping_cells,
    EnvironmentProjections* environment_projections);

void pack_into_proto(
    const std::vector<EnvironmentProjection_CellType>& data,
    const std::vector<int>& workspace_2d_to_surface_3d,
    EnvironmentProjection* projection);

}  // namespace proto
}  // namespace environment
}  // namespace footstep_planner

#endif  // SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_PROTO_ENVIRONMENT_PROJECTIONS_PACK_INTO_PROTO_H_
