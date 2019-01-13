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

#include <footstep_planner/proto/environment_projections/pack_into_proto.h>

namespace footstep_planner {
namespace environment {
namespace proto {

void pack_into_proto(
    const int& lower_workspace_idx,
    const int& upper_workspace_idx,
    WorkspaceIndices* surface_indices) {
    if (lower_workspace_idx > -1) {
        surface_indices->set_lower_workspace_idx(lower_workspace_idx);
    }
    if (upper_workspace_idx > -1) {
        surface_indices->set_upper_workspace_idx(upper_workspace_idx);
    }
}

void pack_into_proto(
    const std::vector<bool>& stepping_cells,
    ValidSteppingCells* valid_stepping_cells_proto) {
    valid_stepping_cells_proto->mutable_stepping_cell()->Reserve(
        stepping_cells.size());
    for (const auto stepping_cell : stepping_cells) {
        valid_stepping_cells_proto->add_stepping_cell(stepping_cell);
    }
}

void pack_into_proto(
    const std::vector<EnvironmentProjection_CellType>& data,
    const std::vector<int>& workspace_2d_to_surface_3d,
    EnvironmentProjection* projection) {
    projection->mutable_data()->Reserve(data.size());
    for (const EnvironmentProjection_CellType cell_type : data) {
        projection->add_data(cell_type);
    }

    projection->mutable_workspace_2d_to_surface_3d()->Reserve(
        workspace_2d_to_surface_3d.size());
    for (const int z_value : workspace_2d_to_surface_3d) {
        projection->add_workspace_2d_to_surface_3d(z_value);
    }
}

void pack_into_proto(
    const int& rows,
    const int& cols,
    const std::vector<EnvironmentProjection>& projections,
    const std::vector<std::pair<int, int>>& surface_indices,
    const std::vector<int>& workspace_3d_to_2d,
    const std::vector<int>& surface_3d_to_stepping_cells,
    EnvironmentProjections* environment_projections) {
    environment_projections->set_rows(rows);
    environment_projections->set_cols(cols);

    environment_projections->mutable_projections()->Reserve(projections.size());
    for (const auto projection : projections) {
        environment_projections->add_projections()->CopyFrom(projection);
    }

    environment_projections->mutable_surface_3d_to_indices()->Reserve(
        surface_indices.size());
    for (const auto pair_indices : surface_indices) {
        pack_into_proto(pair_indices.first, pair_indices.second,
            environment_projections->add_surface_3d_to_indices());
    }

    environment_projections->mutable_workspace_3d_to_2d()->Reserve(
        workspace_3d_to_2d.size());
    for (const int workspace_idx : workspace_3d_to_2d) {
        environment_projections->add_workspace_3d_to_2d(workspace_idx);
    }

    environment_projections->mutable_surface_3d_to_stepping_cells()->Reserve(
        surface_3d_to_stepping_cells.size());
    for (const int stepping_z : surface_3d_to_stepping_cells) {
        environment_projections->add_surface_3d_to_stepping_cells(stepping_z);
    }
}

void pack_into_proto(
    const std::vector<std::vector<proto::EnvironmentProjection_CellType>>& data,
    const std::vector<std::vector<int>>& workspace_2d_to_surface_3d,
    std::vector<proto::EnvironmentProjection>* projection) {
    *projection =
        std::vector<proto::EnvironmentProjection>(
            data.size(),
            proto::EnvironmentProjection());

    for (int i = 0; i < data.size(); ++i) {
        pack_into_proto(
            data[i], workspace_2d_to_surface_3d[i], &((*projection)[i]));
    }
}

}  // namespace proto
}  // namespace environment
}  // namespace footstep_planner
