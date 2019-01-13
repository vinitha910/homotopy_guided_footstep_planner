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

#include <footstep_planner/utils/state_conversions.h>

namespace footstep_planner {
namespace utils {
namespace state_conversions {

double normalize_angle_rad(const double& angle_rad) {
    double normalized_angle_rad = angle_rad;

    // Get the normalized angle in the range of [-2pi, 2pi] if necessary
    if (fabs(normalized_angle_rad) > 2 * M_PI) {
        normalized_angle_rad = normalized_angle_rad - \
        static_cast<int>((normalized_angle_rad / (2 * M_PI))) * 2 * M_PI;
    }

    // Get normalized angle in the range of [0, 2pi] if necessary
    if (angle_rad < 0) normalized_angle_rad += 2 * M_PI;

    assert(normalized_angle_rad >= 0 || normalized_angle_rad <= 2 * M_PI);

    return normalized_angle_rad;
}

double discrete_angle_to_continous(
    const int& theta_rad,
    const int& num_theta_vals) {
    return theta_rad * (2.0 * M_PI / num_theta_vals);
}

int continuous_angle_to_discrete(
    const double& theta_rad,
    const int& num_theta_vals) {
    const double bin_size = 2.0 * M_PI / num_theta_vals;
    const double normalized_angle =
        normalize_angle_rad(theta_rad + bin_size / 2.0) / (2.0 * M_PI) * \
            num_theta_vals;
    return static_cast<int>(normalized_angle);
}

int get_surface(
    const int& id,
    const std::shared_ptr<environment::proto::EnvironmentProjections>
        env_projections) {
    return env_projections->workspace_3d_to_2d(id);
}

}  // namespace state_conversions
}  // namespace utils
}  // namespace footstep_planner
