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

#include <footstep_planner/graphs/nav_lattice_8D.h>
#include <footstep_planner/proto/robot_parameters/unpack_from_proto.h>
#include <footstep_planner/utils/state_conversions.h>
#include <footstep_planner/rviz/visualize.h>
#include <iostream>
#include <cmath>
#include <cassert>

namespace footstep_planner {
namespace graphs {

namespace utils = utils::state_conversions;

NavLattice8D::NavLattice8D(
    const std::shared_ptr<smpl::SparseDistanceMap> distance_map,
    const proto::RobotParameters& robot_details,
    const std::shared_ptr<graphs::HomotopyInformation> homotopy_information,
    const std::shared_ptr<environment::proto::ValidSteppingCells>
        valid_stepping_cells) :
    distance_map_(distance_map),
    homotopy_information_(homotopy_information),
    valid_stepping_cells_(valid_stepping_cells) {
        footstep_planner::proto::unpack_from_proto(
            &robot_parameters_,
            robot_details);
}

NavLattice8D::~NavLattice8D() {
    // Free Foot States
    for (size_t i = 0; i < foot_ID_to_state_.size(); ++i) {
        delete (foot_ID_to_state_[i]);
    }
    foot_ID_to_state_.clear();

    // Free Bipedal States
    for (size_t i = 0; i < bipedal_ID_to_state_.size(); ++i) {
        delete (bipedal_ID_to_state_[i]);
    }
    bipedal_ID_to_state_.clear();

    goal_state_ = NULL;
}

int NavLattice8D::set_bipedal_state(
    const int& x,
    const int& y,
    const int& z,
    const int& theta) {
    double x_m, y_m, z_m;
    distance_map_->gridToWorld(x, y, z, x_m, y_m, z_m);

    // Get the continuous angle normalized to be in the range [0, 2pi]
    double theta_rad =
        utils::normalize_angle_rad(utils::discrete_angle_to_continous(
            theta, robot_parameters_.num_theta_vals));

    const double nominal_offset_m = robot_parameters_.nominal_offset_m;
    const double res = distance_map_->resolution();

    const double right_x_direction = nearbyint(cos(theta_rad));
    const double right_y_direction = nearbyint(sin(theta_rad));
    const Eigen::Vector4d right_foot(
        x_m + (nominal_offset_m/2)*right_x_direction,
        y_m + (nominal_offset_m/2)*right_y_direction,
        z_m,
        theta_rad);

    const double offset_theta_rad =
        utils::normalize_angle_rad(theta_rad + M_PI);
    const double left_x_direction = nearbyint(cos(offset_theta_rad));
    const double left_y_direction = nearbyint(sin(offset_theta_rad));
    const Eigen::Vector4d left_foot(
        x_m + (nominal_offset_m/2)*left_x_direction,
        y_m + (nominal_offset_m/2)*left_y_direction,
        z_m,
        theta_rad);

    FootState* left_foot_state = get_foot_state(left_foot);
    if (left_foot_state == NULL) {
        left_foot_state = create_new_foot_state(left_foot);
        if (!is_valid_foot_state(
                left_foot_state->x,
                left_foot_state->y,
                left_foot_state->z)) {
            ROS_ERROR("[NavLattice8D] Invalid left foot state.");
            return -1;
        }
    }

    FootState* right_foot_state = get_foot_state(right_foot);
    if (right_foot_state == NULL) {
        right_foot_state = create_new_foot_state(right_foot);
        if (!is_valid_foot_state(
                right_foot_state->x,
                right_foot_state->y,
                right_foot_state->z)) {
            ROS_ERROR("[NavLattice8D] Invalid right foot state.");
            return -1;
        }
    }

    BipedalState new_state;
    new_state.next_foot = right;
    new_state.left_foot_id = left_foot_state->id;
    new_state.right_foot_id = right_foot_state->id;
    int signature_id = get_signature_id(std::vector<int>());
    if (signature_id < 0) {
        signature_id = create_new_signature(std::vector<int>());
    }
    new_state.signature_id = signature_id;

    int bipedal_id;
    const auto map_elem = bipedal_state_to_ID_.find(new_state);
    if (map_elem == bipedal_state_to_ID_.end()) {
        bipedal_id = create_new_bipedal_state(new_state);
    } else {
        bipedal_id = map_elem->second;
    }

    if (!is_valid_bipedal_state(bipedal_id)) {
        return -1;
    }

    return bipedal_id;
}

int NavLattice8D::set_start_state(
    const int& x,
    const int& y,
    const int& z,
    const double& theta_rad) {
    const int theta = utils::continuous_angle_to_discrete(
        theta_rad,
        robot_parameters_.num_theta_vals);

    ROS_INFO("[NavLattice8D] Start state: (%d, %d, %d, %d)", x, y, z, theta);

    FootState* start_state = get_foot_state(x, y, z, theta);
    if (start_state == NULL) {
        start_state = create_new_foot_state(x, y, z, theta);
    }

    const int bipedal_id = set_bipedal_state(x, y, z, theta);
    if (bipedal_id == -1) {
        ROS_ERROR("[NavLattice8D] Invalid Bipedal Start State.");
    }

    return bipedal_id;
}

int NavLattice8D::set_goal_state(
    const int& x,
    const int& y,
    const int& z,
    const double& theta_rad) {
    const int theta = utils::continuous_angle_to_discrete(
        theta_rad,
        robot_parameters_.num_theta_vals);

    ROS_INFO("[NavLattice8D] Goal state: (%d, %d, %d, %d)", x, y, z, theta);

    goal_state_ = get_foot_state(x, y, z, theta);
    if (goal_state_ == NULL) {
        goal_state_ = create_new_foot_state(x, y, z, theta);
    }

    const int bipedal_id = set_bipedal_state(x, y, z, theta);
    if (bipedal_id == -1) {
        ROS_ERROR("[NavLattice8D] Invalid Bipedal Goal State.");
    }

    return bipedal_id;
}

int NavLattice8D::get_signature_id(const std::vector<int>& signature) {
    const auto map_elem = signature_to_ID_.find(signature);
    if (map_elem == signature_to_ID_.end()) {
        return -1;
    }
    return map_elem->second;
}

int NavLattice8D::create_new_signature(const std::vector<int>& signature) {
    const int signature_id = ID_to_signature_.size();
    signature_to_ID_[signature] = signature_id;
    ID_to_signature_.push_back(signature);

    return signature_id;
}

void NavLattice8D::get_signature(
    const int& signature_id,
    std::vector<int>* signature) {
    if (signature_id >= ID_to_signature_.size()) {
        ROS_ERROR("[NavLattice8D] Invalid signature ID.");
    }
    *signature = ID_to_signature_[signature_id];
}

bool NavLattice8D::is_goal(const int& current_state_id) {
    const BipedalState* bipedal_state = get_bipedal_state(current_state_id);
    if (bipedal_state == NULL) return false;

    const Eigen::Vector4d center_feet_pos =
        get_cont_averaged_state(
            bipedal_state->left_foot_id,
            bipedal_state->right_foot_id);

    const Eigen::Vector4d goal_pos =
        get_continuous_coordinates(goal_state_->id);
    const double dist_to_goal =
        (center_feet_pos.head<3>() - goal_pos.head<3>()).norm();

    if (dist_to_goal <= robot_parameters_.goal_tolerance_m) {
        return true;
    }

    return false;
}

int NavLattice8D::get_foot_state_id(
    const int& x,
    const int& y,
    const int& z,
    const int& theta) const {
    const int cols = distance_map_->numCellsX();
    const int rows = distance_map_->numCellsY();
    const int height = distance_map_->numCellsZ();
    return (x + (cols * (y + rows * (z + (height * theta)))));
}

int NavLattice8D::get_foot_state_id(
    const double& x_m,
    const double& y_m,
    const double& z_m,
    const double& theta_rad) const {
    int x, y, z;
    distance_map_->worldToGrid(x_m, y_m, z_m, x, y, z);
    const int theta = utils::continuous_angle_to_discrete(
        theta_rad,
        robot_parameters_.num_theta_vals);

    return get_foot_state_id(x, y, z, theta);
}

Eigen::Vector4d NavLattice8D::get_continuous_coordinates(
    const int& state_4D_id) const {
    Eigen::Vector4d cont_coordinates;
    const auto map_elem = foot_ID_to_state_.find(state_4D_id);
    if (map_elem == foot_ID_to_state_.end()) {
        ROS_ERROR("[NavLattice8D] Invalid state ID.");
        return cont_coordinates;
    }

    FootState* state = map_elem->second;
    double x_m, y_m, z_m;
    distance_map_->gridToWorld(state->x, state->y, state->z, x_m, y_m, z_m);

    double theta_rad = utils::discrete_angle_to_continous(
        state->theta, robot_parameters_.num_theta_vals);

    cont_coordinates = Eigen::Vector4d(x_m, y_m, z_m, theta_rad);
    return cont_coordinates;
}

bool NavLattice8D::is_valid_bipedal_state(const int& state_id) {
    std::vector<Eigen::Vector4d> transformed_spheres;
    get_transformed_collision_spheres(state_id, &transformed_spheres);

    for (const auto sphere : transformed_spheres) {
        if (distance_map_->getDistance(
                sphere.x(),
                sphere.y(),
                sphere.z()) <= sphere.w()) {
            return false;
        }
    }

    const BipedalState* bipedal_state = get_bipedal_state(state_id);
    if (bipedal_state == NULL) return false;

    const FootState* left_foot_state =
        get_foot_state(bipedal_state->left_foot_id);
    if (left_foot_state == NULL) {
        ROS_ERROR("[NavLattice8D] Left foot 4D state does not exist.");
        return false;
    }

    const FootState* right_foot_state =
        get_foot_state(bipedal_state->right_foot_id);
    if (right_foot_state == NULL) {
        ROS_ERROR("[NavLattice8D] Right foot 4D state does not exist.");
        return false;
    }

    return true;
}

bool NavLattice8D::is_valid_foot_state(const Eigen::Vector3i& foot_pos) {
    return is_valid_foot_state(foot_pos.x(), foot_pos.y(), foot_pos.z());
}

bool NavLattice8D::is_valid_foot_state(
    const int& x,
    const int& y,
    const int& z) {
    if (distance_map_->getDistance(x, y, z) <= 0) {
        return false;
    }

    const int rows = distance_map_->numCellsY();
    const int cols = distance_map_->numCellsX();
    const int index_3d = cols * (rows * z + y) + x;

    return valid_stepping_cells_->stepping_cell(index_3d);
}

void NavLattice8D::get_transformed_collision_spheres(
    const int &bipedal_state_id,
    std::vector<Eigen::Vector4d>* transformed_spheres) {
    const BipedalState* bipedal_state = get_bipedal_state(bipedal_state_id);
    if (bipedal_state == NULL) return;

    const Eigen::Vector4d left_foot_pos =
        get_continuous_coordinates(bipedal_state->left_foot_id);

    const Eigen::Vector4d right_foot_pos =
        get_continuous_coordinates(bipedal_state->right_foot_id);

    const Eigen::Vector4d center_feet_pos =
        get_cont_averaged_state(
            bipedal_state->left_foot_id,
            bipedal_state->right_foot_id);

    // Check the collision spheres do not intersect anything in the environment
    for (int i = 0; i < robot_parameters_.collision_spheres.size(); i++) {
        Eigen::Vector4d origin_pos = center_feet_pos;
        const auto origin = robot_parameters_.collision_spheres[i].origin;
        if (origin == robot_details::Origin::LEFT_FOOT) {
            origin_pos = left_foot_pos;
        } else if (origin == robot_details::Origin::RIGHT_FOOT) {
            origin_pos = right_foot_pos;
        }

        const Eigen::Vector2d sphere =
            Eigen::Translation2d(origin_pos.x(), origin_pos.y()) * \
            Eigen::Rotation2Dd(origin_pos.w()) * \
            Eigen::Vector2d(robot_parameters_.collision_spheres[i].x,
                            robot_parameters_.collision_spheres[i].y);

        const double sphere_z_m =
            origin_pos.z() + robot_parameters_.collision_spheres[i].z;

        Eigen::Vector4d transformed_sphere(
            sphere.x(),
            sphere.y(),
            sphere_z_m,
            robot_parameters_.collision_spheres[i].radius);

        transformed_spheres->push_back(transformed_sphere);
    }
}

FootState* NavLattice8D::get_foot_state(
    const Eigen::Vector4d& state_pos) const {
    return get_foot_state(
        state_pos.x(),
        state_pos.y(),
        state_pos.z(),
        state_pos.w());
}

FootState* NavLattice8D::get_foot_state(
        const double& x_m,
        const double& y_m,
        const double& z_m,
        const double& theta_rad) const {
    int x, y, z;
    distance_map_->worldToGrid(x_m, y_m, z_m, x, y, z);

    const int theta = utils::continuous_angle_to_discrete(
        theta_rad,
        robot_parameters_.num_theta_vals);

    return get_foot_state(x, y, z, theta);
}

FootState* NavLattice8D::get_foot_state(
    const int& x,
    const int& y,
    const int& z,
    const int& theta) const {
    // Check if cell is in distance map bounds
    if (!distance_map_->isCellValid(x, y, z)) {
        return NULL;
    }

    const int state_id = get_foot_state_id(x, y, z, theta);

    return get_foot_state(state_id);
}

FootState* NavLattice8D::get_foot_state(const int& state_id) const {
    const auto map_elem = foot_ID_to_state_.find(state_id);

    if (map_elem != foot_ID_to_state_.end()) {
        return map_elem->second;
    }

    return NULL;
}

BipedalState* NavLattice8D::get_bipedal_state(const int& state_id) {
    if (state_id >= bipedal_ID_to_state_.size()) {
        ROS_ERROR("[NavLattice8D] Bipedal State does not exist.");
        return NULL;
    }

    return bipedal_ID_to_state_[state_id];
}

FootState* NavLattice8D::create_new_foot_state(
    const Eigen::Vector4d& state_pos) {
    return create_new_foot_state(
        state_pos.x(),
        state_pos.y(),
        state_pos.z(),
        state_pos.w());
}

FootState* NavLattice8D::create_new_foot_state(
    const double& x_m,
    const double& y_m,
    const double& z_m,
    const double& theta_rad) {
    int x, y, z;
    distance_map_->worldToGrid(x_m, y_m, z_m, x, y, z);

    const int theta = utils::continuous_angle_to_discrete(
        theta_rad,
        robot_parameters_.num_theta_vals);

    return create_new_foot_state(x, y, z, theta);
}

FootState* NavLattice8D::create_new_foot_state(
    const int& x,
    const int& y,
    const int& z,
    const int& theta) {
    const int state_id = get_foot_state_id(x, y, z, theta);

    foot_ID_to_state_[state_id] = new FootState;

    foot_ID_to_state_[state_id]->id = state_id;
    foot_ID_to_state_[state_id]->x = x;
    foot_ID_to_state_[state_id]->y = y;
    foot_ID_to_state_[state_id]->z = z;
    foot_ID_to_state_[state_id]->theta = theta;

    return foot_ID_to_state_[state_id];
}

int NavLattice8D::create_new_bipedal_state(const BipedalState& new_state) {
    const int state_id = bipedal_ID_to_state_.size();
    bipedal_ID_to_state_.resize(state_id + 1, nullptr);

    bipedal_ID_to_state_[state_id] = new BipedalState;
    bipedal_ID_to_state_[state_id]->next_foot = new_state.next_foot;
    bipedal_ID_to_state_[state_id]->left_foot_id = new_state.left_foot_id;
    bipedal_ID_to_state_[state_id]->right_foot_id = new_state.right_foot_id;
    bipedal_ID_to_state_[state_id]->signature_id = new_state.signature_id;
    bipedal_state_to_ID_[new_state] = state_id;

    return state_id;
}

double NavLattice8D::get_action_cost(
    const FootState* source_state,
    const FootState* new_state) {
    const Eigen::Vector3d source_pos(
        source_state->x, source_state->y, source_state->z);
    const Eigen::Vector3d new_pos(new_state->x, new_state->y, new_state->z);
    const double norm_val = ((new_pos - source_pos).norm());
    return (norm_val > 0.0001) ? norm_val*100.0 : 100.0;
}

int NavLattice8D::get_active_foot_id(const BipedalState& bipedal_state) {
    if (bipedal_state.next_foot == left) {
        return bipedal_state.left_foot_id;
    }
    return bipedal_state.right_foot_id;
}

int NavLattice8D::get_pivot_foot_id(const BipedalState& bipedal_state) {
    if (bipedal_state.next_foot == left) {
        return bipedal_state.right_foot_id;
    }
    return bipedal_state.left_foot_id;
}

void NavLattice8D::get_foot_succs(
    const BipedalState& bipedal_state,
    std::vector<int>* foot_succs) {
    // Get the pivot foot
    const int pivot_foot_id = get_pivot_foot_id(bipedal_state);
    Eigen::Vector4d pivot_foot = get_continuous_coordinates(pivot_foot_id);

    // If we're pivoting around the right foot, the direction for the
    // pivot point must be offset by 180 degrees
    const Foot pivot_foot_type = bipedal_state.next_foot == left ? right : left;
    const double pivot_direction =
        pivot_foot_type == right ?
        utils::normalize_angle_rad(pivot_foot.w() + M_PI) : pivot_foot.w();

    const double foot_rot_direction = pivot_foot_type == right ? 1.0 : -1.0;

    // Get the active foot
    const int active_foot_id = get_active_foot_id(bipedal_state);
    const Eigen::Vector4d active_foot =
        get_continuous_coordinates(active_foot_id);

    const double eps = 0.00001;
    // Get the pivot point with respect to the active foot (i.e. the foot you
    // want to move).
    const double nominal_offset_m = robot_parameters_.nominal_offset_m;
    const double x_pivot_direction = fabs(cos(pivot_direction)) <= eps ? 0.0 : (
        cos(pivot_direction) < 0 ? -1.0 : 1.0);
    const double y_pivot_direction = fabs(sin(pivot_direction)) <= eps ? 0.0 : (
        sin(pivot_direction) < 0 ? -1.0 : 1.0);
    const Eigen::Vector4d pivot_point(
        pivot_foot.x() + nominal_offset_m*x_pivot_direction,
        pivot_foot.y() + nominal_offset_m*y_pivot_direction,
        active_foot.z(),
        pivot_foot.w());

    // Forward direction of feet is offset by 90 degrees
    const double direction =
        utils::normalize_angle_rad(pivot_foot.w() + M_PI/2);

    const double res = distance_map_->resolution();
    const double x_direction = fabs(cos(direction)) <= eps ? 0.0 : (
        cos(direction) < 0 ? -1.0 : 1.0);
    const double y_direction = fabs(sin(direction)) <= eps ? 0.0 : (
        sin(direction) < 0 ? -1.0 : 1.0);

    for (const auto mprim : robot_parameters_.motion_primitives) {
        const Eigen::Vector4d successor(
            pivot_point.x() + mprim.x()*x_direction*res,
            pivot_point.y() + mprim.y()*y_direction*res,
            pivot_point.z() + mprim.z()*res,
            pivot_point.w() + mprim.w()*foot_rot_direction);

        // Ignore no motion
        if ((successor - active_foot).norm() <= eps) {
            continue;
        }

        FootState* new_foot_state = get_foot_state(successor);
        if (new_foot_state == NULL) {
            new_foot_state = create_new_foot_state(successor);
        }

        if (!is_valid_foot_state(
            new_foot_state->x,
            new_foot_state->y,
            new_foot_state->z)) {
            continue;
        }
        foot_succs->push_back(new_foot_state->id);
    }
}

void NavLattice8D::get_succs(
    const int& bipedal_state_id,
    std::vector<int>* succ_ids,
    std::vector<double>* costs) {
    const BipedalState* bipedal_state = get_bipedal_state(bipedal_state_id);
    if (bipedal_state == NULL) return;

    const int pivot_foot_id = get_pivot_foot_id(*bipedal_state);
    const int active_foot_id = get_active_foot_id(*bipedal_state);

    std::vector<int> foot_succs;
    get_foot_succs(*bipedal_state, &foot_succs);

    for (const int foot_succ_id : foot_succs) {
        const Eigen::Vector4i averaged_state =
            get_disc_averaged_state(pivot_foot_id, foot_succ_id);

        const Eigen::Vector4i parent_avg_state =
            get_disc_averaged_state(
                bipedal_state->left_foot_id,
                bipedal_state->right_foot_id);

        std::vector<int> parent_signature;
        get_signature(bipedal_state->signature_id, &parent_signature);

        std::vector<int> succ_signature;
        homotopy_information_->project_and_get_signature(
            averaged_state.head<3>(),
            parent_avg_state.head<3>(),
            parent_signature,
            &succ_signature);

        int signature_id = get_signature_id(succ_signature);
        if (signature_id < 0) {
            signature_id = create_new_signature(succ_signature);
        }

        BipedalState new_state;
        new_state.next_foot = (bipedal_state->next_foot == left) ? right : left;
        new_state.left_foot_id =
            (bipedal_state->next_foot == left) ? foot_succ_id : pivot_foot_id;
        new_state.right_foot_id =
            (bipedal_state->next_foot == right) ? foot_succ_id : pivot_foot_id;
        new_state.signature_id = signature_id;

        int bipedal_id;
        const auto map_elem = bipedal_state_to_ID_.find(new_state);
        if (map_elem == bipedal_state_to_ID_.end()) {
            bipedal_id = create_new_bipedal_state(new_state);
        } else {
            bipedal_id = map_elem->second;
        }

        // Skip invalid bipedal states
        if (!is_valid_bipedal_state(bipedal_id)) {
            continue;
        }

        const FootState* active_foot = get_foot_state(active_foot_id);
        const FootState* foot_succ = get_foot_state(foot_succ_id);
        const double cost = get_action_cost(active_foot, foot_succ);

        succ_ids->push_back(bipedal_id);
        costs->push_back(cost);
    }
}

Eigen::Vector4d NavLattice8D::get_cont_averaged_state(
    const int& left_state_id,
    const int& right_state_id) const {
    const Eigen::Vector4d left_foot_cont =
        get_continuous_coordinates(left_state_id);
    const Eigen::Vector4d right_foot_cont =
        get_continuous_coordinates(right_state_id);

    Eigen::Vector4d avg_state = (left_foot_cont + right_foot_cont)*0.5;

    const Eigen::Vector2d vec_left(
        cos(left_foot_cont.w()),
        sin(left_foot_cont.w()));

    const Eigen::Vector2d vec_right(
        cos(right_foot_cont.w()),
        sin(right_foot_cont.w()));

    const Eigen::Vector2d vec_center =
        (vec_left != vec_right) ? vec_left + vec_right : vec_left;

    avg_state.w() =
        utils::normalize_angle_rad(atan2(vec_center.y(), vec_center.x()));

    return avg_state;
}

Eigen::Vector4i NavLattice8D::get_disc_averaged_state(
    const int& left_state_id,
    const int& right_state_id) const {
    const Eigen::Vector4d cont_state =
        get_cont_averaged_state(left_state_id, right_state_id);

    int x, y, z;
    distance_map_->worldToGrid(
        cont_state.x(),
        cont_state.y(),
        cont_state.z(),
        x,
        y,
        z);

    const int theta = utils::continuous_angle_to_discrete(
        cont_state.w(),
        robot_parameters_.num_theta_vals);

    const auto avg_state = Eigen::Vector4i(x, y, z, theta);

    return avg_state;
}

}  // namespace graphs
}  // namespace footstep_planner
