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

from proto import homotopy_information_pb2
from point import Point

class HomotopyInformation():
    def __init__(self, beams_filename, gates_filename):
        f = open(beams_filename, 'rb')
        beam_proto = homotopy_information_pb2.Beams()
        beam_proto.ParseFromString(f.read())
        self.beams = beam_proto.beams
        self.workspace_beams = []

        for beam in self.beams:
            while beam.workspace > len(self.workspace_beams) - 1:
                self.workspace_beams.append([])

            self.workspace_beams[beam.workspace].append(beam)

        f = open(gates_filename, 'rb')
        gates_proto = homotopy_information_pb2.Gates()
        gates_proto.ParseFromString(f.read())
        self.gates = gates_proto.gates

        self.signature = []
        self.h_signature = []

    # returns t_k if beam crossed from left-to-right, -t_k beam is crossed from
    # right-to-left and 0 if no beam is crossed
    def beam_direction(self, beam_point, parent_point, successor_point):

        # Crossed beam from left to right
        if parent_point <= beam_point and successor_point > beam_point:
            return 1

        # Crossed beam from right to left
        elif parent_point >= beam_point and successor_point < beam_point:
            return -1

        else:
            return 0

    # Determine if a beam is crossed and if so, which letter to append to the
    # signature
    def get_crossed_beam(self, parent_point, successor_point, gate_workspaces = None):
        beams = self.workspace_beams[parent_point.workspace]
        for beam in beams:
            direction = self.beam_direction(beam, parent_point, successor_point)
            if direction != 0:
                return beam, direction

        beams = self.workspace_beams[successor_point.workspace]
        for beam in beams:
            direction = self.beam_direction(beam, parent_point, successor_point)
            if direction != 0:
                return beam, direction

        if gate_workspaces != None:
            beams = self.workspace_beams[gate_workspaces.lower_workspace_idx]
            for beam in beams:
                direction = self.beam_direction(beam, parent_point, successor_point)
                if direction != 0:
                    return beam, direction

            beams = self.workspace_beams[gate_workspaces.upper_workspace_idx]
            for beam in beams:
                direction = self.beam_direction(beam, parent_point, successor_point)
                if direction != 0:
                    return beam, direction

        return None, 0

    def add_beam(self, beam, direction):
        if len(self.h_signature) == 0 or direction * beam.letter != -self.h_signature[-1]:
            self.h_signature.append(direction * beam.letter)
        elif direction * beam.letter == -self.h_signature[-1]:
            self.h_signature.pop()

    def gate_direction(self, gate, parent_point, successor_point):
        if parent_point.workspace == gate.workspaces.lower_workspace_idx and \
           successor_point.workspace == gate.workspaces.upper_workspace_idx:
            return 1
        elif parent_point.workspace == gate.workspaces.upper_workspace_idx and \
             successor_point.workspace == gate.workspaces.lower_workspace_idx:
            return -1
        else:
            return 0

    def get_crossed_gate(self, parent_point, successor_point):
        for gate_idx in range(len(self.gates)):
            gate = self.gates[gate_idx]
            gate_direction = \
                self.gate_direction(gate, parent_point, successor_point)
            if gate_direction != 0:
                return gate, gate_direction

        return None, 0

    # Appends a new letter to the h-signature and then reduces the h-signature
    def add_gate(self, gate, direction):
        if len(self.h_signature) == 0 or direction * gate.letter != -self.h_signature[-1]:
            self.h_signature.append(direction * gate.letter)
        elif direction * gate.letter == -self.h_signature[-1]:
            self.h_signature.pop()

    # Takes the path points, identifies the beams and gates crossed, and reduces
    # the signatures as new letters are added
    def get_h_signature(self, path_points):
        self.h_signature = []
        # Loop through all points in the path and create the signature
        for pt_idx in range(len(path_points) - 1):
            parent_point = path_points[pt_idx]
            successor_point = path_points[pt_idx + 1]

            crossed_gate, gate_direction = \
                self.get_crossed_gate(parent_point, successor_point)

            gate_workspaces = crossed_gate.workspaces if crossed_gate != None else None
            crossed_beam, beam_direction = \
                self.get_crossed_beam(parent_point, successor_point, gate_workspaces)

            # If the trajectory crossed a beam and gate simultaneously
            if crossed_beam != None and crossed_gate != None:
                # If the beam is on the successor workspace then add/reduce the
                # gate before the beam
                if crossed_beam.workspace == successor_point.workspace:
                    self.add_gate(crossed_gate, gate_direction)
                    self.add_beam(crossed_beam, beam_direction)

                # If the beam is on the parent workspace then add/reduce the
                # beam before the gate
                else:
                    self.add_beam(crossed_beam, beam_direction)
                    self.add_gate(crossed_gate, gate_direction)

            elif crossed_beam == None and crossed_gate != None:
                self.add_gate(crossed_gate, gate_direction)

            elif crossed_beam != None and crossed_gate == None:
                self.add_beam(crossed_beam, beam_direction)

        print "H-signature: " + str(self.h_signature)
        return self.h_signature

    # Create an unreduced signature by taking the path points and identifiying which 
    # beams and gates were crossed and appending the appropriate letter
    def get_signature(self, path_points):
        self.signature = []
        # Loop through all points in the path and create the signature
        for pt_idx in range(len(path_points) - 1):
            parent_point = path_points[pt_idx]
            successor_point = path_points[pt_idx + 1]

            crossed_gate, gate_direction = \
                self.get_crossed_gate(parent_point, successor_point)

            crossed_beam, beam_direction = \
                self.get_crossed_beam(parent_point, successor_point)

            # If the trajectory crossed a beam and gate simultaneously
            if crossed_beam != None and crossed_gate != None:
                # If the beam is on the successor workspace then add/reduce the
                # gate before the beam
                if crossed_beam.workspace == successor_point.workspace:
                    self.signature.append(gate_direction * crossed_gate.letter)
                    self.signature.append(beam_direction * crossed_beam.letter)

                # If the beam is on the parent workspace then add/reduce the
                # beam before the gate
                else:
                    self.signature.append(beam_direction * crossed_beam.letter)
                    self.signature.append(gate_direction * crossed_gate.letter)

            elif crossed_beam == None and crossed_gate != None:
                self.signature.append(gate_direction * crossed_gate.letter)

            elif crossed_beam != None and crossed_gate == None:
                self.signature.append(beam_direction * crossed_beam.letter)

        print "Signature: " + str(self.signature)
        return self.signature
