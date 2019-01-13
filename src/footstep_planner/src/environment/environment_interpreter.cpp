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

#include <footstep_planner/environment/environment_interpreter.h>
#include <footstep_planner/proto/robot_parameters/unpack_from_proto.h>
#include <footstep_planner/proto/environment_projections/pack_into_proto.h>
#include <footstep_planner/utils/helpers.h>
#include <geometric_shapes/mesh_operations.h>
#include <smpl/geometry/voxelize.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <cassert>
#include <cmath>
#include <fstream>

namespace footstep_planner {
namespace environment {

EnvironmentInterpreter::EnvironmentInterpreter(
    const std::string& platforms,
    const std::string& obstacles,
    const std::vector<std::string>& surfaces,
    const std::vector<std::string>& stepping_surfaces,
    const std::map<std::string, double>& map_params,
    const footstep_planner::proto::RobotParameters& robot_details_proto) {
    map_params_ = map_params;

    robot_details::RobotParameters robot_details;
    footstep_planner::proto::unpack_from_proto(
        &robot_details,
        robot_details_proto);

    robot_height_ =
        static_cast<int>(robot_details.height_m/ map_params_["cell_size"]);
    robot_in_radius_m_ = robot_details.in_radius_m;

    max_collision_sphere_radius_ = 0;
    for (int i = 0; i < robot_details.collision_spheres.size(); i++) {
        const double radius = robot_details.collision_spheres[i].radius;
        if (radius > max_collision_sphere_radius_) {
            max_collision_sphere_radius_ = radius;
        }
    }

    std::vector<Eigen::Vector3d> platform_vertices;
    std::vector<Eigen::Vector3d> platform_voxels;
    voxelize_world(platforms, &platform_voxels, &platform_vertices);

    std::vector<Eigen::Vector3d> obstacle_vertices;
    voxelize_world(obstacles, &obstacle_voxels_, &obstacle_vertices);

    std::vector<std::vector<Eigen::Vector3d>> surface_voxels;
    surface_voxels.resize(surfaces.size());
    surface_corners_.resize(surfaces.size());
    for (int i = 0; i < surfaces.size(); i++) {
        voxelize_world(
            surfaces.at(i),
            &surface_voxels.at(i),
            &surface_corners_[i]);
    }

    std::vector<Eigen::Vector3d> stepping_surface_vertices;
    stepping_surface_voxels_.resize(stepping_surfaces.size());
    for (int i = 0; i <stepping_surfaces.size(); i++) {
        voxelize_world(
            stepping_surfaces.at(i),
            &stepping_surface_voxels_.at(i),
            &stepping_surface_vertices);
    }

    for (int i = 0; i < surface_voxels.size(); i++) {
        // The distance map for the specified surface
        const auto surface_distance_map = create_distance_map();
        const std::vector<Eigen::Vector3d> pts = surface_voxels.at(i);
        surface_distance_map->addPointsToMap(pts);
        surface_distance_maps_.push_back(surface_distance_map);
    }

    // The distance map for the full world
    distance_map_ = create_distance_map();
    distance_map_->addPointsToMap(platform_voxels);
    // Remove stepping surface voxels from distance map as they are valid cells
    for (const auto stepping_surface_voxel : stepping_surface_voxels_) {
        distance_map_->removePointsFromMap(stepping_surface_voxel);
    }
    distance_map_->addPointsToMap(obstacle_voxels_);
}

SparseDistanceMapPtr EnvironmentInterpreter::create_distance_map() {
    return std::make_shared<smpl::SparseDistanceMap>(
                map_params_["origin_x"],
                map_params_["origin_y"],
                map_params_["origin_z"],
                map_params_["size_x"],
                map_params_["size_y"],
                map_params_["size_z"],
                map_params_["cell_size"],
                max_collision_sphere_radius_ * 2);
}

void EnvironmentInterpreter::voxelize_world(
    const std::string& mesh,
    std::vector<Eigen::Vector3d>* voxels,
    std::vector<Eigen::Vector3d>* vertices) {
    Eigen::Vector3d voxel_origin(0, 0, 0);

    const shapes::Mesh* m = shapes::createMeshFromResource(mesh);

    vertices->resize(m->vertex_count);
    for (unsigned int i = 0; i < m->vertex_count; ++i) {
        (*vertices)[i] = Eigen::Vector3d(
                m->vertices[3 * i + 0],
                m->vertices[3 * i + 1],
                m->vertices[3 * i + 2]);
    }

    const std::vector<std::uint32_t> indices(
        m->triangles,
        m->triangles + 3 * m->triangle_count);

    smpl::geometry::VoxelizeMesh(
        *vertices,
        indices,
        map_params_["cell_size"],
        voxel_origin,
        *voxels,
        false);
}

void EnvironmentInterpreter::find_line_eq(
    const utils::Coordinates3D& coor1,
    const utils::Coordinates3D& coor2,
    double* m,
    double* b) {
    *m = (coor2.x - coor1.x == 0) ? 0 : (coor2.y - coor1.y)/(coor2.x - coor1.x);
    *b = coor2.y - ((*m)*coor2.x);
}

void EnvironmentInterpreter::find_surface_edges() {
    assert(surface_corners_.size() == surface_distance_maps_.size());

    std::vector<std::vector<double>> surface_edge_lengths;
    for (int surf_idx = 0; surf_idx < surface_corners_.size(); ++surf_idx) {
        const std::vector<Eigen::Vector3d> surface_corners =
            surface_corners_[surf_idx];

        const int num_surface_corners = 4;
        // A surface or plane can only have four corners
        assert(surface_corners.size() == num_surface_corners);

        const std::shared_ptr<smpl::SparseDistanceMap> dist_map =
            surface_distance_maps_[surf_idx];
        std::vector<utils::Coordinates3DPairs> edge_points;

        std::vector<double> edge_lengths;
        // Loop through surface corners and find edge lengths for corner pairs
        for (int i = 0; i < surface_corners.size() - 1; ++i) {
            for (int j = i + 1; j < surface_corners.size(); ++j) {
                const Eigen::Vector3d corner1 = surface_corners[i];
                const Eigen::Vector3d corner2 = surface_corners[j];

                const double edge_length = (corner1 - corner2).norm();
                edge_lengths.push_back(edge_length);

                int x1, y1, z1, x2, y2, z2;
                dist_map->worldToGrid(
                    corner1.x(),
                    corner1.y(),
                    corner1.z(),
                    x1,
                    y1,
                    z1);

                dist_map->worldToGrid(
                    corner2.x(),
                    corner2.y(),
                    corner2.z(),
                    x2,
                    y2,
                    z2);

                const utils::Coordinates3D corner1_grid =
                    {.x = x1, .y = y1, .z = z1};
                const utils::Coordinates3D corner2_grid =
                    {.x = x2, .y = y2, .z = z2};
                edge_points.push_back(
                    std::make_pair(corner1_grid, corner2_grid));
            }
        }

        // Remove diaganol corner pairs
        while (edge_points.size() > num_surface_corners) {
            const auto it =
                std::max_element(edge_lengths.begin(), edge_lengths.end());
            const int idx = std::distance(edge_lengths.begin(), it);

            edge_lengths.erase(it);
            edge_points.erase(edge_points.begin() + idx);
        }

        surface_edge_points_.push_back(edge_points);
    }
}

void EnvironmentInterpreter::create_surface_2d_and_3d_mappings(
    const int& workspace_idx,
    std::vector<std::vector<cell_type_proto>>* projections,
    std::vector<std::vector<int>>* workspace_2d_to_surface_3d,
    std::vector<std::pair<int, int>>* surface_3d_to_indices) {
    const auto surface_distance_map = surface_distance_maps_[workspace_idx];
    const int cols = surface_distance_map->numCellsX();
    const int rows = surface_distance_map->numCellsY();
    const int height = surface_distance_map->numCellsZ();

    (*projections)[workspace_idx] =
        std::vector<cell_type_proto>(
            rows*cols,
            projection_proto::OUTER_WORKSPACE_AREA_CELL);

    // Allocate space for vectors
    (*workspace_2d_to_surface_3d)[workspace_idx] =
        std::vector<int>(rows*cols, -1);

    // Loop through 3d coordinates
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            for (int z = 0; z < height; ++z) {
                // If the (x, y, z) point is a surface voxel
                if (surface_distance_map->getDistance(x, y, z) <= 0) {
                    // Map (x, y, surface) -> (x, y, surface_z)
                    const int idx_2d = y * cols + x;
                    (*workspace_2d_to_surface_3d)[workspace_idx][idx_2d] = z;
                    break;
                }
            }
        }
    }

    const int workspace_2d_to_surface_3d_size =
        (*workspace_2d_to_surface_3d)[workspace_idx].size();
    for (int idx_2d = 0; idx_2d < workspace_2d_to_surface_3d_size; ++idx_2d) {
        const int surface_z =
            (*workspace_2d_to_surface_3d)[workspace_idx][idx_2d];
        if (surface_z == -1) {
            continue;
        }

        const int idx_3d = surface_z * rows * cols + idx_2d;

        // If the lower surface index has been set then
        // set the upper surface index
        if ((*surface_3d_to_indices)[idx_3d].first > -1) {
            // Set gate cells on both surfaces
            (*projections)[workspace_idx][idx_2d] = projection_proto::GATE_CELL;
            const int lower_workspace_idx =
                (*surface_3d_to_indices)[idx_3d].first;
            (*projections)[lower_workspace_idx][idx_2d] =
                projection_proto::GATE_CELL;

            // Set upper surface index
            (*surface_3d_to_indices)[idx_3d].second = workspace_idx;
            continue;
        }

        // If the lower surface index has not been set, set it to
        // surface index and mark the projection cell as free
        (*surface_3d_to_indices)[idx_3d].first = workspace_idx;
        (*projections)[workspace_idx][idx_2d] = projection_proto::FREE_CELL;
    }
}

void EnvironmentInterpreter::project_obstacles(
    const std::vector<std::vector<int>>& workspace_2d_to_surface_3d,
    const std::vector<int>& surface_3d_to_stepping_cells,
    std::vector<std::vector<cell_type_proto>>* projections) {
    std::shared_ptr<smpl::SparseDistanceMap> obstacle_distance_map =
        create_distance_map();
    obstacle_distance_map->addPointsToMap(obstacle_voxels_);

    const int rows = obstacle_distance_map->numCellsY();
    const int cols = obstacle_distance_map->numCellsX();
    const int height = obstacle_distance_map->numCellsZ();
    const int num_surfaces = projections->size();

    for (int y = 0; y < rows;  ++y) {
        for (int x = 0; x < cols; ++x) {
            for (int surface = 0; surface < num_surfaces; ++surface) {
                // Go from (x, y, workspace_idx) -> (x, y, surface_z)
                const int idx_2d = y * cols + x;
                const int surface_z =
                    workspace_2d_to_surface_3d[surface][idx_2d];

                if (surface_z == -1) {
                    continue;
                }

                // Go from (x, y, surface_z) -> (x, y, stepping_z)
                const int idx_3d = surface_z * rows * cols + idx_2d;
                const int stepping_z = surface_3d_to_stepping_cells[idx_3d];

                if (stepping_z == height) continue;

                // See if there are any obstacles above the stepping cell
                for (int z = stepping_z; z <= stepping_z + robot_height_; ++z) {
                    if (obstacle_distance_map->getDistance(x, y, z) <= 0 &&
                        ((*projections)[surface][idx_2d] == \
                            projection_proto::FREE_CELL ||
                         (*projections)[surface][idx_2d] == \
                            projection_proto::GATE_CELL)) {
                        (*projections)[surface][idx_2d] = \
                            projection_proto::OBSTACLE_CELL;
                        break;
                    }
                }
            }
        }
    }
}

void EnvironmentInterpreter::compute_distance_transform(
    const std::vector<std::vector<cell_type_proto>>& projections,
    std::vector<int>* distance_transform_2d) {
    const int cols = distance_map_->numCellsX();
    const int rows = distance_map_->numCellsY();
    const int num_surfaces = projections.size();
    *distance_transform_2d = std::vector<int>(rows*cols*num_surfaces, 0);
    int G[cols][rows][num_surfaces];

    const int inf = cols + rows + num_surfaces;

    // scan the first row
    for (int surface = 0; surface < num_surfaces; ++surface) {
        const auto projection = projections.size();
        for (int x = 0; x < cols; ++x) {
            const bool is_obstacle =
                projections[surface][x] == projection_proto::OBSTACLE_CELL;
            const bool is_surface_boundary =
                projections[surface][x] == \
                    projection_proto::WORKSPACE_BOUNDARY_CELL;
            if (is_obstacle || is_surface_boundary) {
                G[x][0][surface] = 0;
            } else {
                G[x][0][surface] = inf;
            }

            for (int y = 1; y < rows; ++y) {
                const int idx = y*cols + x;
                if (is_obstacle || is_surface_boundary) {
                    G[x][y][surface] = 0;
                } else {
                    G[x][y][surface] = G[x][y - 1][surface] + 1;
                }
            }

            for (int y = rows - 2; y >= 0; --y) {
                if (G[x][y + 1][surface] < G[x][y][surface]) {
                    G[x][y][surface] = 1 + G[x][y + 1][surface];
                }
            }
        }
    }

    float val;
    for (int surface = 0; surface < num_surfaces; ++surface) {
        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                int m = INFINITECOST, d;
                for (int i = 0; i < cols; ++i) {
                    d = utils::sqrd(x - i) + utils::sqrd(G[i][y][surface]);
                    if (d < m) {
                        m = d;
                    }
                }
                const int idx = cols*(rows*surface + y) + x;
                (*distance_transform_2d)[idx] = std::sqrt(m);
            }
        }
    }
}

void EnvironmentInterpreter::inflate_obstacles(
    std::vector<std::vector<cell_type_proto>>* projections) {
    std::vector<int> distance_transform_2d;
    compute_distance_transform(*projections, &distance_transform_2d);

    const double inflation_radius =
        robot_in_radius_m_/distance_map_->resolution();
    const int num_surfaces = projections->size();

    // Loop through every surface
    for (int surface = 0; surface < num_surfaces; ++surface) {
        const auto projection = (*projections)[surface];
        const int rows = distance_map_->numCellsY();
        const int cols = distance_map_->numCellsX();

        // Loop through every point on the surface
        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                const int idx_2d = cols * y + x;
                const int idx = cols * rows * surface + idx_2d;
                const int dist = distance_transform_2d[idx];

                // If the distance to the nearest obstacle/surface boundary cell
                // is less than the inflation radius, mark the cell as either an
                // obstacle cell or surface boundary cell appropiately
                if (dist <= robot_in_radius_m_) {
                    const bool is_obstacle =
                        ((*projections)[surface][idx_2d] ==
                            projection_proto::OBSTACLE_CELL);
                    const auto cell_type =
                         is_obstacle ? projection_proto::OBSTACLE_CELL : \
                         projection_proto::WORKSPACE_BOUNDARY_CELL;
                    (*projections)[surface][idx_2d] = cell_type;
                }
            }
        }
    }
}

void EnvironmentInterpreter::project_surface_boundaries(
    const int& workspace_idx,
    const std::vector<utils::Coordinates3DPairs>& edge_points,
    std::vector<cell_type_proto>* projection) {
    const auto surface_distance_map = surface_distance_maps_[workspace_idx];

    // If cell at the given index is not a gate cell, mark it as a boundary cell
    auto set_boundary = [&](int idx) {
        if ((*projection)[idx] != projection_proto::GATE_CELL) {
            (*projection)[idx] = projection_proto::WORKSPACE_BOUNDARY_CELL;
        }
    };

    for (const auto edge_point_pair : edge_points) {
        const int x_min =
            std::min(edge_point_pair.first.x, edge_point_pair.second.x);
        const int x_max =
            std::max(edge_point_pair.first.x, edge_point_pair.second.x);
        const int y_min =
            std::min(edge_point_pair.first.y, edge_point_pair.second.y);
        const int y_max =
            std::max(edge_point_pair.first.y, edge_point_pair.second.y);

        // If the edge is a horizontal line (i.e. slope = 0)
        if (y_min == y_max) {
            for (int x = x_min; x <= x_max; ++x) {
                set_boundary(y_min * surface_distance_map->numCellsX() + x);
            }

        // If the edge is a vertical line (i.e. no slope)
        } else if (x_min == x_max) {
            for (int y = y_min; y <= y_max; ++y) {
                set_boundary(y * surface_distance_map->numCellsX() + x_min);
            }

        // If the edge has a slope that does not equal 0
        } else {
            double m, b;
            find_line_eq(edge_point_pair.first, edge_point_pair.second, &m, &b);
            for (int x = x_min; x <= x_max; ++x) {
                const int y = m*x + b;
                set_boundary(y * surface_distance_map->numCellsX() + x);
            }
        }
    }
}

void EnvironmentInterpreter::create_valid_stepping_cells_mapping(
    std::vector<bool>* stepping_cells) {
    std::shared_ptr<smpl::SparseDistanceMap> stepping_cells_distance_map =
        create_distance_map();

    for (const auto stepping_surface_voxel : stepping_surface_voxels_) {
        stepping_cells_distance_map->addPointsToMap(stepping_surface_voxel);
    }

    const int cols = stepping_cells_distance_map->numCellsX();
    const int rows = stepping_cells_distance_map->numCellsY();
    const int height = stepping_cells_distance_map->numCellsZ();
    *stepping_cells = std::vector<bool>(rows * cols * height, false);

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            for (int z = 0; z < height; ++z) {
                if (stepping_cells_distance_map->getDistance(x, y, z) <= 0) {
                    const int index = cols * (z * rows + y) + x;
                    (*stepping_cells)[index] = true;
                }
            }
        }
    }
}


void EnvironmentInterpreter::create_workspace_3d_to_2d_mapping(
    std::vector<int>* workspace_3d_to_2d,
    std::vector<std::vector<int>>* workspace_2d_to_surface_3d) {
    const int cols = distance_map_->numCellsX();
    const int rows = distance_map_->numCellsY();
    const int height = distance_map_->numCellsZ();

    *workspace_3d_to_2d = std::vector<int>(rows*cols*height, 0);
    for (int z = 0; z < height; ++z) {
        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                const int index = cols * (rows * z + y) + x;
                const int index_2d = cols * y + x;

                int min_surface = -1;
                int min_dist_to_surface = height;
                for (int i = 0; i < workspace_2d_to_surface_3d->size(); ++i) {
                    const int surface_z =
                        (*workspace_2d_to_surface_3d)[i][index_2d];
                    if (surface_z == -1) {
                        continue;
                    }
                    const int dist_to_surf = abs(z - surface_z);
                    if (dist_to_surf <= min_dist_to_surface) {
                        min_dist_to_surface = dist_to_surf;
                        min_surface = i;
                    }
                }
                (*workspace_3d_to_2d)[index] = min_surface;
            }
        }
    }
}

void EnvironmentInterpreter::create_surface_3d_to_stepping_cells_mapping(
    const std::vector<std::vector<int>>& workspace_2d_to_surface_3d,
    const std::vector<bool>& stepping_cells,
    std::vector<int>* surface_3d_to_stepping_cells) {
    const int cols = distance_map_->numCellsX();
    const int rows = distance_map_->numCellsY();
    const int height = distance_map_->numCellsZ();
    const int num_surfaces = surface_distance_maps_.size();

    *surface_3d_to_stepping_cells = std::vector<int>(rows*cols*height, -1);
    for (int surface = 0; surface < num_surfaces; ++surface) {
        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                const int idx_2d = y * cols + x;
                const int surface_z =
                    workspace_2d_to_surface_3d[surface][idx_2d];

                if (surface_z == -1) {
                    continue;
                }
                // Default the closest stepping cell to be invalid
                int closest_stepping_cell_z = height;
                int closest_stepping_cell_dist = height;

                // Iterate over all platform z values to find stepping cells
                for (int z = surface_z; z < height; ++z) {
                    const int stepping_cell_idx = z * rows * cols + idx_2d;

                    if (stepping_cells[stepping_cell_idx]) {
                        const int dist = std::abs(surface_z -z);
                        if (dist < closest_stepping_cell_dist) {
                            closest_stepping_cell_z = z;
                            closest_stepping_cell_dist = dist;
                        }
                    }
                }

                // Write the stepping cell as the closest stepping cell to
                // (x, y, surface_z)
                const int surface_index_3d = surface_z * rows * cols + idx_2d;
                (*surface_3d_to_stepping_cells)[surface_index_3d] =
                    closest_stepping_cell_z;
            }
        }
    }
}

void EnvironmentInterpreter::project_environment() {
    find_surface_edges();

    const int num_surfaces = surface_distance_maps_.size();
    std::vector<std::vector<cell_type_proto>> projections
        = std::vector<std::vector<cell_type_proto>>(
              num_surfaces,
              std::vector<cell_type_proto>());

    std::vector<std::vector<int>> workspace_2d_to_surface_3d =
        std::vector<std::vector<int>> (
            num_surfaces,
            std::vector<int>());

    const int rows = distance_map_->numCellsY();
    const int cols = distance_map_->numCellsX();
    const int height = distance_map_->numCellsZ();

    std::vector<std::pair<int, int>> surface_3d_to_indices =
        std::vector<std::pair<int, int>>(
            rows*cols*height,
            std::pair<int, int>(-1, -1));

    std::vector<bool> stepping_cells;
    create_valid_stepping_cells_mapping(&stepping_cells);

    // Loop through the distance map for each surface
    for (int workspace_idx = 0; workspace_idx < num_surfaces; ++workspace_idx) {
        create_surface_2d_and_3d_mappings(
            workspace_idx,
            &projections,
            &workspace_2d_to_surface_3d,
            &surface_3d_to_indices);

        // Project the surface boundaries for the specified surface
        const auto edge_points = surface_edge_points_[workspace_idx];
        project_surface_boundaries(
            workspace_idx,
            edge_points,
            &projections[workspace_idx]);
    }

    std::vector<int> workspace_3d_to_2d;
    create_workspace_3d_to_2d_mapping(
        &workspace_3d_to_2d,
        &workspace_2d_to_surface_3d);

    std::vector<int> surface_3d_to_stepping_cells;
    create_surface_3d_to_stepping_cells_mapping(
        workspace_2d_to_surface_3d,
        stepping_cells,
        &surface_3d_to_stepping_cells);

    project_obstacles(
        workspace_2d_to_surface_3d,
        surface_3d_to_stepping_cells,
        &projections);
    inflate_obstacles(&projections);

    std::vector<proto::EnvironmentProjection> projections_proto;
    pack_into_proto(
        projections,
        workspace_2d_to_surface_3d,
        &projections_proto);

    pack_into_proto(
        distance_map_->numCellsY(),
        distance_map_->numCellsX(),
        projections_proto,
        surface_3d_to_indices,
        workspace_3d_to_2d,
        surface_3d_to_stepping_cells,
        &env_projections_);

    env_projections_ptr_ =
        std::make_shared<proto::EnvironmentProjections>(env_projections_);

    pack_into_proto(stepping_cells, &stepping_cells_);

    stepping_cells_ptr_ =
        std::make_shared<proto::ValidSteppingCells>(stepping_cells_);
}

bool EnvironmentInterpreter::project_and_save_environment(
    const char* projection_out_file,
    const char* stepping_cells_out_file) {
    project_environment();
    std::fstream projections_output(projection_out_file, std::ios::out |
        std::ios::trunc | std::ios::binary);
    if (!env_projections_.SerializeToOstream(&projections_output)) {
        ROS_ERROR("Error: failed to serialize EnvironmentProjections protbuf");
        return false;
    }

    std::fstream stepping_cells_output(stepping_cells_out_file, std::ios::out |
        std::ios::trunc | std::ios::binary);
    if (!stepping_cells_.SerializeToOstream(&stepping_cells_output)) {
        ROS_ERROR("Error: failed to serialize ValidSteppingCells protbuf");
        return false;
    }
    return true;
}

bool EnvironmentInterpreter::read_projection(
    const char* projection_in_file,
    const char* stepping_cells_in_file) {
    // Read the protobuf message
    std::fstream projections_input(projection_in_file, std::ios::in |
        std::ios::binary);
    if (!env_projections_.ParseFromIstream(&projections_input)) {
        ROS_ERROR("Error: failed to parse EnvironmentProjections protbuf");
        return false;
    }

    std::fstream stepping_cells_input(stepping_cells_in_file, std::ios::in |
        std::ios::binary);
    if (!stepping_cells_.ParseFromIstream(&stepping_cells_input)) {
        ROS_ERROR("Error: failed to parse ValidSteppingCells protbuf");
        return false;
    }

    env_projections_ptr_ = std::make_shared<proto::EnvironmentProjections>(
        env_projections_);

    stepping_cells_ptr_ = std::make_shared<proto::ValidSteppingCells>(
        stepping_cells_);

    return true;
}
}  // namespace environment
}  // namespace footstep_planner
