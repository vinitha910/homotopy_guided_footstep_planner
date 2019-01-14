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

import matplotlib
matplotlib.use('GTKAgg')

import sys
from graphs.point import Point
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib import colors
from random import randint, uniform
import yaml
import math
import glob
import numpy as np
import re

sys.setrecursionlimit(10000)

class UserInterface():
    def __init__(self, projections, homotopy_information,
        environment, batches=[]):

        self.environment =  environment
        self.projections = projections
        self.homotopy_information = homotopy_information
        self.batches = batches

        # For fixed start and goal pairs, remove this line and set the 
        # self.start_x, self.goal_x etc. properties manually
        self.generate_random_start_goal_pairs()

        self.start_circle = \
            plt.Circle((self.goal_x,self.goal_y), 2, color='green')
        self.goal_circle = \
            plt.Circle((self.start_x,self.start_y), 2, color='red')
        self.intermediate_circle = None

        # Make a color map of fixed colors
        self.cmap = colors.ListedColormap(
            ['grey', 'white', 'orange', 'black', 'black'])
        bounds=[1,2, 3, 4, 5]
        norm = colors.BoundaryNorm(bounds, self.cmap.N)

        self.current_workspace = self.goal_workspace

        self.batch_environments()

        self.fig, self.ax = plt.subplots(figsize=(10,10))

        self.axb1 = plt.axes([0.123, 0.03, 0.15, 0.05])
        self.sigb1 = Button(self.axb1, 'Add Signature')
        self.sigb1.on_clicked(self.add_signature)

        self.axb2 = plt.axes([0.3, 0.03, 0.13, 0.05])
        self.sigb2 = Button(self.axb2, 'Delete Path')
        self.sigb2.on_clicked(self.delete_path)

        self.axb3 = plt.axes([0.45, 0.03, 0.265, 0.05])
        self.sigb3 = Button(self.axb3, 'Regenerate Start/Goal Pairs')
        self.sigb3.on_clicked(self.regenerate_random_start_goal_pairs)

        self.axb4 = plt.axes([0.74, 0.03, 0.158, 0.05])
        self.sigb4 = Button(self.axb4, 'Save Test File')
        self.sigb4.on_clicked(self.write_test_file)

        self.goal_circle_artist = None
        self.intermediate_circle_artist = None
        self.start_circle_artist = self.ax.add_artist(self.start_circle)
        if (self.start_workspace in self.current_batch or
            self.start_workspace == self.current_workspace):
            self.goal_circle_artist = self.ax.add_artist(self.goal_circle)

        self.ax.axis('off')
        self.cid = self.fig.canvas.mpl_connect(
            'motion_notify_event', self.onclick)
        self.cidrelease = self.fig.canvas.mpl_connect(
            'button_release_event', self.on_release)

        self.ax.imshow(self.map_values, cmap=self.cmap)
        self.ax.set_ylim([0, self.map_values.shape[0]])
        self.ax.set_xlim([1, self.map_values.shape[1]])
        self.ax.set_title("Draw a path from the green circle to the red circle")

        self.xy = []
        self.path_points = []
        self.points = self.points = self.ax.scatter([], [], s=5, color='blue', picker=20)
        self.h_signatures = []
        self.signatures = []

        plt.show()

    # Determine which workspaces should be displayed simultaneously
    def batch_environments(self):
        self.current_batch = []
        in_batch = [self.current_workspace in batch for batch in self.batches]
        if any(in_batch):
            self.current_batch_idx = in_batch.index(True)
            self.current_batch = self.batches[self.current_batch_idx]

        if len(self.current_batch) == 0:
            self.map_values = \
                self.projections.get_workspace_map(self.current_workspace)
            return

        self.map_values = self.projections.get_workspace_map(self.current_batch[0])
        if len(self.current_batch) > 1:
            for workspace in self.current_batch[1:]:
                self.map_values = np.maximum.reduce(
                    [self.projections.get_workspace_map(workspace),
                    self.map_values])

    def onclick(self, event):
        if event.button and event.inaxes:
            x = int(event.xdata)
            y = int(event.ydata)

            last_x = int(event.xdata)
            last_y = int(event.ydata)
            if len(self.xy) is not 0:
                last_x = self.xy[-1][0]
                last_y = self.xy[-1][1]

            # Interpolating points fills in the missing path points caused by
            # lag in matplotlib
            interpolated_pts = zip\
                (np.linspace(last_x, event.xdata, 50),\
                 np.linspace(last_y, event.ydata, 50))
            self.xy.extend(interpolated_pts)

            last_workspace = -1
            if len(self.path_points) > 0:
                last_workspace = self.path_points[-1].workspace

            workspaces = \
                self.projections.get_current_workspace(x, y, self.current_batch)
            if len(workspaces) > 1 and last_workspace in workspaces:
                self.path_points.append(Point(x, y, last_workspace))
            else:
                for workspace in workspaces:
                    self.path_points.append(Point(x, y, workspace))

            self.points.set_offsets(self.xy)
            self.ax.draw_artist(self.points)

            self.fig.canvas.blit(self.ax.bbox)
            self.fig.canvas.flush_events()

    # Refresh the map that is being displayed if the cursor is released over a 
    # gate. 
    def on_release(self, event):
        x = int(event.xdata)
        y = int(event.ydata)
        workspaces = self.projections.get_current_workspace(x, y, self.current_batch)
        if len(workspaces) > 0: self.current_workspace = workspaces[0]

        if self.projections.is_gate_cell(x, y, self.current_workspace):
            workspace_indices = \
                self.projections.get_workspace_indices(x, y, self.current_workspace)

            # Redraw the start circle from where the user ended the path on the
            # previous workspace
            self.intermediate_circle = plt.Circle((x, y), 2, color='blue')
            self.ax.set_title("Draw a path from the blue circle to the red circle")

            if self.current_workspace == workspace_indices.lower_workspace_idx:
                self.redraw_gui(workspace_indices.upper_workspace_idx)
            elif self.current_workspace == workspace_indices.upper_workspace_idx:
                self.redraw_gui(workspace_indices.lower_workspace_idx)

            
    # Generate random start and goal points on free cells in a random workspace
    def generate_random_start_goal_pairs(self):
        valid_start = False
        valid_goal = False

        rows, cols = \
                self.projections.get_projection_size()

        while not valid_start:
            self.start_workspace = \
                randint(0, self.projections.num_projections - 1)

            self.start_x = randint(0, cols - 1)
            self.start_y = randint(0, rows - 1)
            self.start_theta = round(uniform(0, 2*math.pi),3)

            if self.projections.is_free_cell(
                self.start_x, self.start_y, self.start_workspace):
                valid_start = True

        while not valid_goal:
            self.goal_workspace = randint(0, self.projections.num_projections - 1)

            self.goal_x = randint(0, cols - 1)
            self.goal_y = randint(0, rows - 1)
            self.goal_theta = round(uniform(0, 2*math.pi),3)

            if self.projections.is_free_cell(
                self.goal_x, self.goal_y, self.goal_workspace):
                    valid_goal = True

    # Takes the path_points list and maps it to a signature and h-signature
    # using the functions defined in homotopy_information.py
    def add_signature(self, event):
        signature = self.homotopy_information.get_signature(self.path_points)
        self.signatures.append(signature)

        h_signature = self.homotopy_information.get_h_signature(self.path_points)
        self.h_signatures.append(h_signature)

        self.path_points = []
        self.delete_path(event)

    def reset_circle_artists(self):
        if self.start_circle_artist:
            self.start_circle_artist.remove()
            self.start_circle_artist = None

        if self.goal_circle_artist:
            self.goal_circle_artist.remove()
            self.goal_circle_artist = None

        if self.intermediate_circle_artist:
            self.intermediate_circle_artist.remove()
            self.intermediate_circle_artist = None

        self.start_circle = \
            plt.Circle((self.goal_x,self.goal_y), 2, color='green')
        self.goal_circle = \
            plt.Circle((self.start_x,self.start_y), 2, color='red')
        self.intermediate_circle = None
        self.ax.set_title("Draw a path from the green circle to the red circle")
        self.fig.canvas.flush_events()

    def regenerate_random_start_goal_pairs(self, event):
        self.generate_random_start_goal_pairs()
        self.reset_circle_artists()
        self.path_points = []
        self.h_signatures = []
        self.signatures = []
        self.redraw_gui(self.goal_workspace)

    def delete_path(self, event):
        self.path_points = []
        self.reset_circle_artists()
        self.redraw_gui(self.goal_workspace)

    # update the workspace as well as start and goal circles
    def redraw_gui(self, workspace):
        self.current_workspace = workspace
        self.batch_environments()

        self.ax.imshow(self.map_values, cmap=self.cmap)
        self.ax.set_ylim([0, self.map_values.shape[0]])
        self.ax.set_xlim([1, self.map_values.shape[1]])

        if (self.goal_workspace in self.current_batch or
            self.goal_workspace == self.current_workspace):
            self.start_circle_artist = self.ax.add_artist(self.start_circle)
        elif self.start_circle_artist:
            self.start_circle_artist.remove()
            self.start_circle_artist = None

        if (self.start_workspace in self.current_batch or
            self.start_workspace == self.current_workspace):
            self.goal_circle_artist = self.ax.add_artist(self.goal_circle)
        elif self.goal_circle_artist:
            self.goal_circle_artist.remove()
            self.goal_circle_artist = None

        if self.intermediate_circle_artist:
            self.intermediate_circle_artist.remove()
            self.intermediate_circle_artist = None

        if self.intermediate_circle:
            self.intermediate_circle_artist = \
                self.ax.add_artist(self.intermediate_circle)

        self.xy = []
        self.points.set_offsets(self.xy)
        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.draw()

        self.fig.canvas.flush_events()

    def sorted_alphanumeric_strings(self, l):
        """ Sorts the given iterable in the way that is expected.

        Required arguments:
        l -- The iterable to be sorted.

        """
        convert = lambda text: int(text) if text.isdigit() else text
        alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
        return sorted(l, key = alphanum_key)

    # Write out a .yaml file containing homotopy information from demonstrations
    def write_test_file(self, event):
        test_num = (len(glob.glob("../scenarios/hbsp_1/*"))) + 1

        # Files to write the 1st h-signature, all h-signatures, 
        # and no h-signatures respectively
        filename_hbsp_1 = "../scenarios/hbsp_1/hbsp_test_" + str(test_num) + "_1.yaml"
        filename_hbsp_3 = "../scenarios/hbsp_3/hbsp_test_" + str(test_num) + "_3.yaml"
        hbsp_filenames = [filename_hbsp_1, filename_hbsp_3]

        filename_dijkstra = \
            "../scenarios/dijkstra/dijkstra_test_" + str(test_num) + ".yaml"

        workspace_path = "/worlds/" + self.environment + "/workspaces/"
        workspaces = np.sort(glob.glob(".." + workspace_path + "*"))
        surf_string = ["package://footstep_planner" + surf[2:] \
            for surf in workspaces]
        surf_string = str(self.sorted_alphanumeric_strings(surf_string))

        stepping_surf_path = "/worlds/" + self.environment + "/stepping_workspaces/"
        stepping_workspaces = np.sort(glob.glob(".." + stepping_surf_path + "*"))
        stepping_surf_string = ["package://footstep_planner" + surf[2:] \
            for surf in stepping_workspaces]
        stepping_surf_string = str(self.sorted_alphanumeric_strings(stepping_surf_string))

        # query_type is used when comparing 'simple' and 'complex queries'
        query_type = raw_input("Query Type: ")

        num_signatures = 1
        for filename_hbsp in hbsp_filenames:
            f1 = open(filename_hbsp, "w+")
            f1.write('frames: ["poppy_frame"]\n')
            f1.write('\n')
            f1.write('query_type: "' + query_type + '"\n')
            f1.write('\n')
            f1.write('surfaces: ' + surf_string + '\n')
            f1.write('stepping_surfaces: ' + stepping_surf_string + '\n')
            f1.write('obstacles: "package://footstep_planner/worlds/house/obstacles.stl"\n')
            f1.write('platforms: "package://footstep_planner/worlds/house/platforms.stl"\n')
            f1.write("\n")
            f1.write('load_proto_msgs: true\n')
            f1.write("\n")
            f1.write("proto_msgs:\n")
            f1.write('    package: "footstep_planner"\n')
            f1.write('    robot_parameters: "/proto/robot_parameters.msg"\n')
            f1.write('    projections: "/proto/projections.msg"\n')
            f1.write('    stepping_cells: "/proto/valid_stepping_cells.msg"\n')
            f1.write('    beams: "/proto/beams.msg"\n')
            f1.write('    gates: "/proto/gates.msg"\n')
            f1.write("\n")
            f1.write("signatures: " +
                str(self.signatures[:num_signatures]) + "\n")
            f1.write("h_signatures: " +
                str(self.h_signatures[:num_signatures]) + "\n")
            f1.write("\n")
            f1.write("visualize:\n")
            f1.write("    robot_model: false\n")
            f1.write("    world: false\n")
            f1.write("    collision_model: false\n")
            f1.write("    anchor_heuristic: false\n")
            f1.write("    homotopic_heuristic: false\n")
            f1.write("    expansions: false\n")
            f1.write("    path: false\n")
            f1.write("\n")
            f1.write("map_params:\n")
            f1.write("    size_x: 23.8\n")
            f1.write("    size_y: 15.3\n")
            f1.write("    size_z: 5.2\n")
            f1.write("    origin_x: 0.0\n")
            f1.write("    origin_y: 0.0\n")
            f1.write("    origin_z: 0.0\n")
            f1.write("    cell_size: 0.1\n")
            f1.write("\n")
            f1.write("start_pose:\n")
            f1.write("    x: " + str(self.start_x) + "\n")
            f1.write("    y: " + str(self.start_y) + "\n")
            f1.write("    workspace: " + str(self.start_workspace) + "\n")
            f1.write("    theta: " + str(self.start_theta) + "\n")
            f1.write("\n")
            f1.write("goal_pose:\n")
            f1.write("    x: " + str(self.goal_x) + "\n")
            f1.write("    y: " + str(self.goal_y) + "\n")
            f1.write("    workspace: " + str(self.goal_workspace) + "\n")
            f1.write("    theta: " + str(self.goal_theta) + "\n")
            f1.close()
            print("Saved " + filename_hbsp)
            num_signatures += 2

        f2 = open(filename_dijkstra, "w+")
        f2.write('frames: ["poppy_frame"]\n')
        f2.write('\n')
        f2.write('query_type: "' + query_type + '"\n')
        f2.write('\n')
        f2.write('surfaces: ' + surf_string + '\n')
        f2.write('stepping_surfaces: ' + stepping_surf_string + '\n')
        f2.write('obstacles: "package://footstep_planner/worlds/house/obstacles.stl"\n')
        f2.write('platforms: "package://footstep_planner/worlds/house/platforms.stl"\n')
        f2.write("\n")
        f2.write('load_proto_msgs: true\n')
        f2.write("\n")
        f2.write("proto_msgs:\n")
        f2.write('    package: "footstep_planner"\n')
        f2.write('    robot_parameters: "/proto/robot_parameters.msg"\n')
        f2.write('    projections: "/proto/projections.msg"\n')
        f2.write('    stepping_cells: "/proto/valid_stepping_cells.msg"\n')
        f2.write('    beams: "/proto/beams.msg"\n')
        f2.write('    gates: "/proto/gates.msg"\n')
        f2.write("\n")
        f2.write("signatures: []\n")
        f2.write("h_signatures: []\n")
        f2.write("\n")
        f2.write("visualize:\n")
        f2.write("    robot_model: false\n")
        f2.write("    world: false\n")
        f2.write("    collision_model: false\n")
        f2.write("    anchor_heuristic: false\n")
        f2.write("    homotopic_heuristic: false\n")
        f2.write("    expansions: false\n")
        f2.write("    path: false\n")
        f2.write("\n")
        f2.write("map_params:\n")
        f2.write("    size_x: 23.8\n")
        f2.write("    size_y: 15.3\n")
        f2.write("    size_z: 5.2\n")
        f2.write("    origin_x: 0.0\n")
        f2.write("    origin_y: 0.0\n")
        f2.write("    origin_z: 0.0\n")
        f2.write("    cell_size: 0.1\n")
        f2.write("\n")
        f2.write("start_pose:\n")
        f2.write("    x: " + str(self.start_x) + "\n")
        f2.write("    y: " + str(self.start_y) + "\n")
        f2.write("    workspace: " + str(self.start_workspace) + "\n")
        f2.write("    theta: " + str(self.start_theta) + "\n")
        f2.write("\n")
        f2.write("goal_pose:\n")
        f2.write("    x: " + str(self.goal_x) + "\n")
        f2.write("    y: " + str(self.goal_y) + "\n")
        f2.write("    workspace: " + str(self.goal_workspace) + "\n")
        f2.write("    theta: " + str(self.goal_theta) + "\n")
        f2.close()
        print("Saved " + filename_dijkstra)
