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
#Initializes interfaces to simulations of robot navigation, pan-tilt unit and object localization.

sleep 5
# wait for gazebo
# while [[ ! $(rosservice list | grep gaz) ]]; do sleep 1; done;
# while [[ ! $(xwininfo -root -all | grep rviz) ]]; do sleep 1; done;

#Start ROS navigation stack for simulating MILD robot.
tmux new-window -n 'move_mock'
tmux send-keys -t asr:move_mock 'roslaunch --wait asr_mild_navigation simulation_manual_rearranged.launch'  C-m

#Basic wrapper for ptu driver simulation.
tmux new-window -n 'ptu_mock'
tmux send-keys -t asr:ptu_mock 'roslaunch --wait asr_flir_ptu_driver ptu_left_mock.launch'  C-m
#Action server for interfacing with ptu in state_machine.
tmux split-window -h -t asr:ptu_mock
tmux send-keys -t asr:ptu_mock.1 'sleep 10; roslaunch --wait asr_flir_ptu_controller ptu_controller.launch'  C-m
#Script to control action server by hand.
tmux split-window -v -t asr:ptu_mock
tmux send-keys -t asr:ptu_mock.2 'sleep 10; rfasr="rospack find asr_resources_for_active_scene_recognition"; path=$(eval $rfasr);  roscd asr_flir_ptu_controller/src/;python cli_controll.py -v $path"/src/getFrustum.py"'  C-m

#Script to trigger RViz visualization of robot frustum. Necessary to set ptu angles to that objects are in initial view of robot for ASR standalone mode.
tmux split-window -h -t asr:ptu_mock
tmux send-keys -t asr:ptu_mock.3 'sleep 10; roscd asr_resources_for_active_scene_recognition/src/; python getFrustum.py'  C-m
# so we can use arrow up to refresh the frustum
tmux send-keys -t asr:ptu_mock.4 'roscd asr_resources_for_active_scene_recognition/src/; python getFrustum.py'  C-m

#Starts object localization simulation for all real recognizers we have on MILD.
tmux new-window -n 'fake_recognizer'
tmux send-keys -t asr:fake_recognizer 'roslaunch --wait asr_fake_object_recognition fake_object_recognition.launch' C-m
