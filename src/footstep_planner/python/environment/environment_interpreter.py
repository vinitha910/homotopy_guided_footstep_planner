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

from proto import environment_projections_pb2
import numpy as np

class EnvironmentInterpreter():
    def __init__(self, projections_filename):
        f = open(projections_filename, 'rb')
        self.environment_projections = \
            environment_projections_pb2.EnvironmentProjections()
        self.environment_projections.ParseFromString(f.read())

        self.num_projections = len(self.environment_projections.projections)

    # searches through a batch of workspaces to find one containing the 
    # corresponding (x, y)
    def get_current_workspace(self, x, y, batch):
        workspaces = []
        for workspace in batch:
            if not self.is_outer_workspace_area_cell(x, y, workspace):
                workspaces.append(workspace)
        if len(workspaces) > 0:
            return workspaces
        return [-1]

    # Builds a numpy array of all the cells in a workspace for rendering
    def get_workspace_map(self, workspace):
        projection = self.environment_projections.projections[workspace];
        data = list()
        for cell in projection.data:
            data.append(cell)
        data = np.reshape(np.asarray(data), (self.environment_projections.rows, -1))
        return data

    # Maps an (x, y, workspace) tuple to the surface(s) it belongs to
    # if the (x, y, workspace) is a gate then there are two surfaces
    def get_workspace_indices(self, x, y, workspace):
        projection = self.environment_projections.projections[workspace]
        cols = self.environment_projections.cols
        rows = self.environment_projections.rows
        workspace_2d_idx = y*cols + x
        surface_z = projection.workspace_2d_to_surface_3d[workspace_2d_idx]
        surface_3d_idx = (surface_z * rows * cols) + (y * cols) + x;
        return self.environment_projections.surface_3d_to_indices[surface_3d_idx]

    # returns the type of cell at (x, y, workspace)
    def get_projected_cell(self, x, y, workspace):
        projection = self.environment_projections.projections[workspace]
        idx = int(y)*self.environment_projections.cols + int(x)
        return projection.data[idx]

    def get_projection_size(self):
        return self.environment_projections.rows, self.environment_projections.cols

    def is_outer_workspace_area_cell(self, x, y, workspace):
        cell = self.get_projected_cell(x, y, workspace)
        return cell == environment_projections_pb2.EnvironmentProjection.OUTER_WORKSPACE_AREA_CELL

    def is_gate_cell(self, x, y, workspace):
        cell = self.get_projected_cell(x, y, workspace)
        return cell == environment_projections_pb2.EnvironmentProjection.GATE_CELL

    def is_free_cell(self, x, y, workspace):
        cell = self.get_projected_cell(x, y, workspace)
        return cell == environment_projections_pb2.EnvironmentProjection.FREE_CELL
