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

#ifndef SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_PROTO_HOMOTOPY_INFORMATION_UNPACK_FROM_PROTO_H_
#define SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_PROTO_HOMOTOPY_INFORMATION_UNPACK_FROM_PROTO_H_

#include <footstep_planner/utils/helpers.h>
#include <homotopy_information.pb.h>
#include <vector>
#include <utility>
#include <set>

namespace footstep_planner {
namespace graphs {
namespace proto {

utils::Coordinates2D unpack_from_proto(const Beams_Beam& beam);

void unpack_from_proto(
    std::vector<std::set<utils::Coordinates2D>>* beams,
    const Beams& beams_proto);

std::pair<int, int> unpack_from_proto(const Gates_Gate& gate);

void unpack_from_proto(
    std::set<std::pair<int, int>>* gates,
    const Gates& gates_proto);

}  // namespace proto
}  // namespace graphs
}  // namespace footstep_planner

#endif  // SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_PROTO_HOMOTOPY_INFORMATION_UNPACK_FROM_PROTO_H_
