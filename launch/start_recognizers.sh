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
#Script that starts all object localization systems for various scene functions that robot is required to provide. Separated from rest because segmentable_recognition cannot be started in remote shell.

#Start tmux session if not already existing.
if ! { [ "$TERM" = "screen" ] && [ -n "$TMUX" ]; } then
  tmux new-session -d -s asr
  tmux send-keys 'roscd resources_for_active_scene_recognition/launch/; ./start_recognizers.sh' C-m
  tmux attach
  exit 1
else
  tmux rename-session asr
fi

#Object localization (segmentable and textured) from Pedram Azad.
tmux rename-window 'recognition_for_grasping'
tmux send-keys -t asr:recognition_for_grasping 'roslaunch --wait recognition_for_grasping recognition_manager_core.launch' C-m

#New object localization, based on Halcon lib.
tmux new-window -n 'descriptor_surface_based_recognition'
tmux send-keys  -t asr:descriptor_surface_based_recognition 'roslaunch --wait asr_descriptor_surface_based_recognition descriptor_surface_based_recognition_core.launch' C-m

#Localization of markers, based on ArUco lib
tmux new-window -n 'aruco_marker_recognition' 
tmux send-keys -t asr:aruco_marker_recognition 'roslaunch --wait asr_aruco_marker_recognition aruco_marker_recognition_core.launch' C-m

#Object database that contains models, trained for object localization and models for visualization.
tmux new-window -n 'object_database'
tmux send-keys -t asr:object_database 'rosrun asr_object_database asr_object_database' C-m


