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

#include <footstep_planner/proto/robot_parameters/unpack_from_proto.h>

namespace footstep_planner {
namespace proto {

void unpack_from_proto(
    robot_details::CollisionSpheres* collision_sphere,
    const CollisionSpheres& collision_spheres_proto) {
    collision_sphere->x = collision_spheres_proto.x();
    collision_sphere->y = collision_spheres_proto.y();
    collision_sphere->z = collision_spheres_proto.z();
    collision_sphere->radius = collision_spheres_proto.radius();

    if (collision_spheres_proto.origin() == CollisionSpheres::LEFT_FOOT) {
        collision_sphere->origin = robot_details::LEFT_FOOT;
    } else if (
        collision_spheres_proto.origin() == CollisionSpheres::RIGHT_FOOT) {
        collision_sphere->origin = robot_details::RIGHT_FOOT;
    } else {
        collision_sphere->origin = robot_details::FEET_CENTER;
    }
}

void unpack_from_proto(
    Eigen::Vector4d* motion_primitive,
    const MotionPrimitives& motion_primitive_proto) {
    motion_primitive->x() = motion_primitive_proto.dx();
    motion_primitive->y() = motion_primitive_proto.dy();
    motion_primitive->z() = motion_primitive_proto.dz();
    motion_primitive->w() = 0.0;
}

void unpack_from_proto(
    robot_details::RobotParameters* robot_parameters,
    const RobotParameters& robot_parameters_proto) {
    robot_parameters->height_m = robot_parameters_proto.height_m();
    robot_parameters->in_radius_m =  robot_parameters_proto.in_radius_m();
    robot_parameters->foot_length_m =
        robot_parameters_proto.foot_length_m();
    robot_parameters->nominal_offset_m =
        robot_parameters_proto.nominal_offset_m();
    robot_parameters->num_theta_vals =
        robot_parameters_proto.num_theta_vals();
    robot_parameters->goal_tolerance_m =
        robot_parameters_proto.goal_tolerance_m();

    robot_parameters->collision_spheres.reserve(
        robot_parameters_proto.collision_spheres_size());
    robot_parameters->motion_primitives.reserve(
        robot_parameters_proto.motion_primitives_size());

    for (int i = 0; i < robot_parameters_proto.collision_spheres_size(); i++) {
        robot_details::CollisionSpheres collision_sphere;
        unpack_from_proto(
            &collision_sphere,
            robot_parameters_proto.collision_spheres(i));
        robot_parameters->collision_spheres.push_back(collision_sphere);
    }

    for (int i = 0; i < robot_parameters_proto.motion_primitives_size(); i++) {
        Eigen::Vector4d mprim;
        unpack_from_proto(
            &mprim,
            robot_parameters_proto.motion_primitives(i));
        robot_parameters->motion_primitives.push_back(mprim);
    }

    robot_parameters->motion_primitives.push_back(
        Eigen::Vector4d(
            0.0,
            0.0,
            0.0,
            2.0 * M_PI / robot_parameters->num_theta_vals));
}

}  // namespace proto
}  // namespace footstep_planner
