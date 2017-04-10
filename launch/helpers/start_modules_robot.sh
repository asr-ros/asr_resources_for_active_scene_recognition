#!/bin/bash

#Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Karrenbauer Oliver, Marek Felix, Meissner Pascal, Stroh Daniel, Trautmann Jeremias
#All rights reserved.
#
#Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
#1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
#2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other #materials provided with the distribution.
#
#3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific #prior written permission.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED #WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, #INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR #PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) #ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# read http://unix.stackexchange.com/questions/17116/prevent-pane-window-from-closing-when-command-completes-tmux
#Initializes interfaces to cameras, pan-tilt unit and robot navigation.

tmux new-window -n 'move'
#Controlling of robot base is required to be started separately on i61r7.
tmux send-keys -t asr:move.0 'echo roslaunch mild_base_launch_files mild.launch Please start controlling of robot base manually on i61r7; ssh odete@i61r7' C-m
tmux split-window -t asr:move
#ROS navigation stack with setting for MILD robot.
tmux send-keys -t asr:move.1 'roslaunch --wait asr_mild_navigation navigation.launch' C-m

#Basic wrapper for ptu driver.
tmux new-window -n 'ptu'
tmux send-keys -t asr:ptu.0 'roslaunch --wait asr_flir_ptu_driver ptu_left.launch' C-m 
#Action server for interfacing with ptu in state_machine.
tmux split-window -h -t asr:ptu
tmux send-keys -t asr:ptu.1 'sleep 40;roslaunch --wait asr_flir_ptu_controller ptu_controller.launch' C-m
#Script to control action server by hand.
tmux split-window -v -t asr:ptu
tmux send-keys -t asr:ptu.2 'sleep 40;roscd asr_flir_ptu_controller/src/;python cli_controll.py' C-m

#Starts stereo camera setup with calibration files for guppy cameras on MILD.
tmux new-window -n 'vision'
tmux send-keys -t asr:vision 'roslaunch --wait asr_resources_for_vision guppy_head_full_mild.launch' C-m

#Wrapper for kinect drivers and part of tf tree towards kinect.
tmux new-window -n 'kinect'
tmux send-keys -t asr:kinect.0 'roslaunch --wait asr_mild_kinematic_chain transformation_publishers_kinect_left.launch' C-m
tmux split-window -t asr:kinect
tmux send-keys -t asr:kinect.1 'roslaunch --wait asr_resources_for_vision kinect_with_registered_guppy_mild.launch' C-m



