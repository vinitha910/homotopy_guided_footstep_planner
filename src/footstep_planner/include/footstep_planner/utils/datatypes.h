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

#ifndef SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_UTILS_DATATYPES_H_
#define SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_UTILS_DATATYPES_H_

namespace footstep_planner {
namespace graphs {

// A 2D graph state
struct State2D {
    int x;
    int y;
    int workspace;

    bool operator==(const State2D &other) const {
        return (x == other.x &&
                y == other.y &&
                workspace == other.workspace);
    }
};

// An h-augmented vertex
struct Vertex {
    int id;
    int signature_id;

    bool operator==(const Vertex &other) const {
        return (id == other.id &&
                signature_id == other.signature_id);
    }
};

// The 4D state of the foot
struct FootState {
    int id;
    int x;
    int y;
    int z;
    int theta;

    bool operator==(const FootState &other) const {
        return (x == other.x &&
                y == other.y &&
                z == other.z &&
                theta == other.theta);
    }
};

enum Foot { left, right };

struct BipedalState {
    // The next foot that needs to be expanded
    Foot next_foot;

    // The FootState ID of the left foot
    int left_foot_id;

    // The FootState ID of the right foot
    int right_foot_id;

    // The signature for this Bipedal State; the signature is calculated
    // using the averaged state
    int signature_id;

    bool operator==(const BipedalState &other) const {
        return (next_foot == other.next_foot &&
                left_foot_id == other.left_foot_id &&
                right_foot_id == other.right_foot_id &&
                signature_id == other.signature_id);
    }
};

}  // namespace graphs
}  // namespace footstep_planner

namespace std {

template<>
struct hash<footstep_planner::graphs::Vertex> {
    inline size_t operator()(const footstep_planner::graphs::Vertex& v) const {
        return boost::hash_value(v.id);
    }
};

template<>
struct hash<footstep_planner::graphs::BipedalState> {
    inline size_t operator()(const footstep_planner::graphs::BipedalState& b)
        const {
        size_t hash = 0;
        boost::hash_combine(hash, b.next_foot);
        boost::hash_combine(hash, b.left_foot_id);
        boost::hash_combine(hash, b.right_foot_id);
        boost::hash_combine(hash, b.signature_id);
        return hash;
    }
};

}  // namespace std

#endif  // SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_UTILS_DATATYPES_H_
