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

#ifndef SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_UTILS_STATE_CONVERSIONS_H_
#define SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_UTILS_STATE_CONVERSIONS_H_

#include <environment_projections.pb.h>
#include <iostream>
#include <memory>

namespace footstep_planner {
namespace utils {
namespace state_conversions {

// This function take an angle (radians) and outputs an angle in the
// range [0, 2pi] (assume counterclockwise rotation is positive)
double normalize_angle_rad(const double& angle_rad);

// This function converts a discretized angle into a continuous angle (radians)
double discrete_angle_to_continous(
    const int& theta_rad,
    const int& num_theta_vals);

// This function converts a continous angle (radians) into a discrete angle
int continuous_angle_to_discrete(
    const double& theta_rad,
    const int& num_theta_vals);

// This function projects (x, y, platform_z) -> (x, y, workspace_idx)
// Returns the surface index
int get_surface(
    const int& id,
    const std::shared_ptr<environment::proto::EnvironmentProjections>
        env_projections);

}  // namespace state_conversions
}  // namespace utils
}  // namespace footstep_planner

#endif  // SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_UTILS_STATE_CONVERSIONS_H_
