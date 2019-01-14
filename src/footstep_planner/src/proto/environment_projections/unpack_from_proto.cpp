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

#include <footstep_planner/proto/environment_projections/unpack_from_proto.h>

namespace footstep_planner {
namespace environment {
namespace proto {

CellType unpack_from_proto(
    const EnvironmentProjection_CellType proto_cell_type) {
    if (proto_cell_type == EnvironmentProjection::OUTER_WORKSPACE_AREA_CELL) {
        return OUTER_WORKSPACE_AREA_CELL;
    } else if (proto_cell_type == EnvironmentProjection::FREE_CELL) {
        return FREE_CELL;
    } else if (proto_cell_type == EnvironmentProjection::GATE_CELL) {
        return GATE_CELL;
    } else if (proto_cell_type == EnvironmentProjection::OBSTACLE_CELL) {
        return OBSTACLE_CELL;
    } else {
        return WORKSPACE_BOUNDARY_CELL;
    }
}

void unpack_from_proto(
    int* lower_workspace_idx,
    int* upper_workspace_idx,
    const WorkspaceIndices& surface_indices) {
    *lower_workspace_idx =
        surface_indices.has_lower_workspace_idx() ? \
        surface_indices.lower_workspace_idx() : -1;

    *upper_workspace_idx =
        surface_indices.has_upper_workspace_idx() ? \
        surface_indices.upper_workspace_idx() : -1;
}

void unpack_from_proto(
    std::vector<bool>* stepping_cells,
    const ValidSteppingCells& valid_stepping_cells_proto) {
    stepping_cells->reserve(valid_stepping_cells_proto.stepping_cell_size());
    for (int i = 0; i < valid_stepping_cells_proto.stepping_cell_size(); ++i) {
        stepping_cells->push_back(valid_stepping_cells_proto.stepping_cell(i));
    }
}

void unpack_from_proto(
    std::vector<CellType>* data,
    std::vector<int>* workspace_2d_to_surface_3d,
    const EnvironmentProjection& projection) {
    data->reserve(projection.data_size());
    for (int i = 0; i < projection.data_size(); ++i) {
        data->push_back(unpack_from_proto(projection.data(i)));
    }

    workspace_2d_to_surface_3d->reserve(
        projection.workspace_2d_to_surface_3d_size());
    for (int i = 0; i < projection.workspace_2d_to_surface_3d_size(); ++i) {
        workspace_2d_to_surface_3d->push_back(
            projection.workspace_2d_to_surface_3d(i));
    }
}

void unpack_from_proto(
    std::vector<std::vector<CellType>>* data,
    std::vector<std::vector<int>>* workspace_2d_to_surface_3d,
    const std::vector<proto::EnvironmentProjection>& projection) {
    *data = std::vector<std::vector<CellType>>(
        projection.size(),
        std::vector<CellType>());

    for (int i = 0; i < projection.size(); i++) {
        (*data)[i] = std::vector<CellType>(projection[i].data_size());
        for (int j = 0; j < projection[i].data_size(); j++) {
            (*data)[i][j] = unpack_from_proto(projection[i].data(j));
        }
    }

    (*workspace_2d_to_surface_3d) = std::vector<std::vector<int>>(
        projection.size(),
        std::vector<int>());
    for (int i = 0; i < projection.size(); ++i) {
        const int workspace_2d_to_surface_3d_size =
            projection[i].workspace_2d_to_surface_3d_size();
        (*workspace_2d_to_surface_3d)[i] =
            std::vector<int>(workspace_2d_to_surface_3d_size);
        for (int j = 0; j < workspace_2d_to_surface_3d_size; ++j) {
            (*workspace_2d_to_surface_3d)[i][j] =
                projection[i].workspace_2d_to_surface_3d(j);
        }
    }
}

void unpack_from_proto(
    std::vector<std::pair<int, int>>* surface_indices,
    const std::shared_ptr<EnvironmentProjections> environment_projections) {
    const auto surface_indices_size =
        environment_projections->surface_3d_to_indices_size();
    *surface_indices =
        std::vector<std::pair<int, int>>(
            surface_indices_size,
            std::pair<int, int>());
    for (int i = 0; i < surface_indices_size; ++i) {
        int lower, upper;
        unpack_from_proto(
            &lower,
            &upper,
            environment_projections->surface_3d_to_indices(i));
        (*surface_indices)[i] = std::make_pair(lower, upper);
    }
}

void unpack_from_proto(
    std::vector<int>* workspace_3d_to_2d,
    const std::shared_ptr<EnvironmentProjections> environment_projections) {
    const int workspace_3d_to_2d_size =
        environment_projections->workspace_3d_to_2d_size();
    *workspace_3d_to_2d =
        std::vector<int>(
            workspace_3d_to_2d_size);
    for (int i = 0; i < workspace_3d_to_2d_size; ++i) {
        (*workspace_3d_to_2d)[i] =
            environment_projections->workspace_3d_to_2d(i);
    }
}

void unpack_from_proto(
    std::vector<EnvironmentProjection>* projections,
    std::vector<std::pair<int, int>>* surface_indices,
    std::vector<int>* workspace_3d_to_2d,
    std::vector<int>* surface_3d_to_stepping_cells,
    const std::shared_ptr<EnvironmentProjections> environment_projections) {
    const auto projections_size = environment_projections->projections_size();
    *projections =
        std::vector<EnvironmentProjection>(
            projections_size,
            EnvironmentProjection());
    for (int i = 0; i < projections_size; i++) {
        projections->push_back(environment_projections->projections(i));
    }

    unpack_from_proto(surface_indices, environment_projections);
    unpack_from_proto(workspace_3d_to_2d, environment_projections);

    const auto surface_3d_to_stepping_cells_size =
        environment_projections->surface_3d_to_indices_size();
    *surface_3d_to_stepping_cells =
        std::vector<int>(surface_3d_to_stepping_cells_size);
    for (int i = 0; i < surface_3d_to_stepping_cells_size; ++i) {
        surface_3d_to_stepping_cells->push_back(
            environment_projections->surface_3d_to_stepping_cells(i));
    }
}

void unpack_from_proto(
    std::vector<graphs::Projection>* projections,
    const std::shared_ptr<EnvironmentProjections> environment_projections) {

    const int projections_size = environment_projections->projections_size();
    for (int surface = 0; surface < projections_size; ++surface) {
        const auto projection_proto =
            environment_projections->projections(surface);
        projections->reserve(environment_projections->projections_size());
        graphs::Projection projection;
        unpack_from_proto(
            &projection.data,
            &projection.workspace_2d_to_surface_3d,
            projection_proto);

        projections->push_back(projection);
    }
}

void unpack_from_proto(
    int* rows,
    int* cols,
    std::vector<graphs::Projection>* projections,
    const std::shared_ptr<EnvironmentProjections> environment_projections) {
    *rows = environment_projections->rows();
    *cols = environment_projections->cols();

    unpack_from_proto(projections, environment_projections);
}

}  // namespace proto
}  // namespace environment
}  // namespace footstep_planner
