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

#ifndef SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_PROTO_ENVIRONMENT_PROJECTIONS_UNPACK_FROM_PROTO_H_
#define SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_PROTO_ENVIRONMENT_PROJECTIONS_UNPACK_FROM_PROTO_H_

#include <footstep_planner/utils/helpers.h>
#include <environment_projections.pb.h>
#include <utility>
#include <vector>

namespace footstep_planner {
namespace environment {
namespace proto {

CellType unpack_from_proto(
    const EnvironmentProjection_CellType proto_cell_type);

void unpack_from_proto(
    int* lower_workspace_idx,
    int* upper_workspace_idx,
    const WorkspaceIndices& surface_indices);

void unpack_from_proto(
    std::vector<bool>* stepping_cells,
    const ValidSteppingCells& valid_stepping_cells_proto);

void unpack_from_proto(
    std::vector<CellType>* data,
    std::vector<int>* workspace_2d_to_surface_3d,
    const EnvironmentProjection& projection);

void unpack_from_proto(
    std::vector<std::vector<CellType>>* data,
    std::vector<std::vector<int>>* workspace_2d_to_surface_3d,
    const std::vector<proto::EnvironmentProjection>& projection);

void unpack_from_proto(
    std::vector<std::pair<int, int>>* surface_indices,
    const std::shared_ptr<EnvironmentProjections> environment_projections);

void unpack_from_proto(
    std::vector<int>* workspace_3d_to_2d,
    const std::shared_ptr<EnvironmentProjections> environment_projections);

void unpack_from_proto(
    std::vector<EnvironmentProjection>* projections,
    std::vector<std::pair<int, int>>* surface_indices,
    std::vector<int>* workspace_3d_to_2d,
    std::vector<int>* surface_3d_to_stepping_cells,
    const std::shared_ptr<EnvironmentProjections> environment_projections);

void unpack_from_proto(
    std::vector<graphs::Projection>* projections,
    const std::shared_ptr<EnvironmentProjections> environment_projections);

void unpack_from_proto(
    int* rows,
    int* cols,
    std::vector<graphs::Projection>* projections,
    const std::shared_ptr<EnvironmentProjections> environment_projections);

}  // namespace proto
}  // namespace environment
}  // namespace footstep_planner

#endif  // SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_PROTO_ENVIRONMENT_PROJECTIONS_UNPACK_FROM_PROTO_H_
