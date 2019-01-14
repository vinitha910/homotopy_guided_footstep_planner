################################################################################
## Copyright (c) 2019, Vinitha Ranganeni & Sahit Chintalapudi
## All rights reserved.
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are met:
##
##     1. Redistributions of source code must retain the above copyright notice
##        this list of conditions and the following disclaimer.
##     2. Redistributions in binary form must reproduce the above copyright
##        notice, this list of conditions and the following disclaimer in the
##        documentation and/or other materials provided with the distribution.
##     3. Neither the name of the copyright holder nor the names of its
##        contributors may be used to endorse or promote products derived from
##        this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
## AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
## ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
## LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
## CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
## SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
## INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
## CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
## ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
## POSSIBILITY OF SUCH DAMAGE.
################################################################################

import argparse
import matplotlib.pyplot as plt
from matplotlib import colors
from environment.environment_interpreter import EnvironmentInterpreter

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Create scenarios for testing the homotopy-based planner')
    parser.add_argument('projections_message_path',
                        help='The path to the projections protobuf message')
    parser.add_argument('workspace_index',
                        help='The index of the projected workspace to visualize')
    args = parser.parse_args()

    projections = EnvironmentInterpreter(args.projections_message_path)
    map_values = projections.get_workspace_map(int(args.workspace_index))
    fig, ax = plt.subplots(figsize=(10,10))

    cmap = colors.ListedColormap(
        ['grey', 'white', 'orange', 'black', 'black'])
    bounds=[1, 2, 3, 4, 5]
    norm = colors.BoundaryNorm(bounds, cmap.N)

    ax.imshow(map_values, cmap=cmap)
    plt.ylim([0, map_values.shape[0]])
    plt.xlim([1, map_values.shape[1]])
    ax.axis('off')
    plt.show()
