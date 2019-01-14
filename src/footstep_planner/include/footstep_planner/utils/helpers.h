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

#ifndef SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_UTILS_HELPERS_H_
#define SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_UTILS_HELPERS_H_

#include <Eigen/Geometry>
#include <utility>
#include <functional>
#include <algorithm>
#include <vector>
#include <boost/functional/hash.hpp>

namespace footstep_planner {

namespace environment {

enum CellType {
    OUTER_WORKSPACE_AREA_CELL,
    FREE_CELL,
    GATE_CELL,
    OBSTACLE_CELL,
    WORKSPACE_BOUNDARY_CELL
};

}  // namespace environment

namespace graphs {

struct Projection {
    std::vector<environment::CellType> data;
    std::vector<int> workspace_2d_to_surface_3d;
};

}  // namespace graphs

namespace robot_details {

enum Origin {
    LEFT_FOOT,
    RIGHT_FOOT,
    FEET_CENTER,
};

struct CollisionSpheres {
    double x;
    double y;
    double z;
    double radius;
    Origin origin;
};

struct RobotParameters {
    double height_m;
    double in_radius_m;
    double foot_length_m;
    double foot_width_m;
    double nominal_offset_m;
    int num_theta_vals;
    double goal_tolerance_m;
    std::vector<CollisionSpheres> collision_spheres;
    std::vector<Eigen::Vector4d> motion_primitives;
};

}  // namespace robot_details

namespace utils {

#define INFINITECOST 1000000000

template <class T>
T sqrd(T a) { return a * a; }

struct Coordinates2D {
    int x;
    int y;

    bool operator==(const Coordinates2D &other) const {
        return (x == other.x && y == other.y);
    }

    bool operator<(const Coordinates2D& c) const {
        return(x < c.x);
    }
};

struct Coordinates3D {
    int x;
    int y;
    int z;

    bool operator==(const Coordinates3D &other) const {
        return (x == other.x && y == other.y && z == other.z);
    }

    bool operator<(const Coordinates3D& c) const {
        return(x < c.x && y < c.y && z < c.z);
    }
};

typedef std::pair<Coordinates3D, Coordinates3D> Coordinates3DPairs;

}  // namespace utils
}  // namespace footstep_planner

namespace std {

// Custom hash function for Coordinates2D
template<>
struct hash<footstep_planner::utils::Coordinates2D> {
    inline size_t operator()(
        const footstep_planner::utils::Coordinates2D& coor) const {
        std::size_t seed = 0;
        boost::hash_combine(seed, coor.x);
        boost::hash_combine(seed, coor.y);
        return seed;
    }
};

// Custom comparator for Coordinates3DPairs
template<>
struct less<footstep_planner::utils::Coordinates3DPairs> {
    inline size_t operator()(
        const footstep_planner::utils::Coordinates3DPairs& c1,
        const footstep_planner::utils::Coordinates3DPairs& c2) const {
        const int min_c1_x = std::min(c1.first.x, c1.second.x);
        const int min_c2_x = std::min(c2.first.x, c2.second.x);
        return(min_c1_x < min_c2_x);
    }
};

// Custom hash function for vector
template<>
struct hash<std::vector<int>> {
    inline size_t operator()(const std::vector<int>& v) const {
        return boost::hash_range(v.begin(), v.end());
    }
};

}  // namespace std

#endif  // SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_UTILS_HELPERS_H_
