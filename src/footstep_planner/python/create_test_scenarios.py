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

from environment.environment_interpreter import EnvironmentInterpreter
from graphs.homotopy_information import HomotopyInformation
from interfaces.user_interface import UserInterface
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Create scenarios for testing the homotopy-based planner')
    parser.add_argument('projections_message_path',
                        help='The path to the projections protobuf message')
    parser.add_argument('beams_message_path',
                        help='The path to the beams protobuf message')
    parser.add_argument('gates_message_path',
                        help='The path to the gates protobuf message')
    parser.add_argument('environment',
                        help='name of the environment folder (house or toy)')
    parser.add_argument('batches', nargs='?', default=[],
                        help='A list of lists of workspace indices you would like to visualize at the same time')
    args = parser.parse_args()
    projections = EnvironmentInterpreter(args.projections_message_path)
    homotopy_information = HomotopyInformation(
        args.beams_message_path, args.gates_message_path)

    if len(args.batches) > 0:
        str_batches = args.batches.replace('[','').split('],')
        batches = [map(int, batch.replace(']','').split(',')) for batch in str_batches]
        UserInterface(projections, homotopy_information, args.environment, batches)
    else:
        UserInterface(projections, homotopy_information, args.environment, args.batches)
