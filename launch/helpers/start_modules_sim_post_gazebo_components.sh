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
# Starts additional simulation modules that cannot be launched before gazebo is running

# wait for gazebo and rviz

sleep 5

# while [[ ! $(rosservice list | grep gaz) ]]; do sleep 1; done;
# while [[ ! $(xwininfo -root -all | grep rviz) ]]; do sleep 1; done;


# make sure log folder exist, so all modules start safely
export logFolder=~/log
mkdir -p ${logFolder}

#Starts scene recognition, pose prediction and provides a pane for interfaces with object localization simulation.
tmux new-window -n 'ism' 
tmux send-keys -t asr:ism 'script -c "roslaunch --wait recognizer_prediction_ism rp_ism_node.launch" -f '"${logFolder}"'/ism.log' C-m
tmux split-window -t asr:ism
tmux send-keys -t asr:ism.1 'echo Perform service calls to fake_object_recognition from here.' C-m

#Starts next-best-view calculation and world model.
tmux new-window -n 'nbv'
tmux send-keys -t asr:nbv 'script -c "roslaunch --wait asr_next_best_view next_best_view_core_sim.launch" -f '"${logFolder}"'/nbv.log' C-m
tmux split-window -t asr:nbv
tmux send-keys -t asr:nbv.1 'script -c "roslaunch --wait asr_world_model world_model.launch" -f '"${logFolder}"'/world_model.log' C-m

#Starts visualization server to publish the room model.
tmux new-window -n 'viz_server'
tmux send-keys -t asr:viz_server 'script -c "roslaunch --wait asr_visualization_server visualization.launch" -f '"${logFolder}"'/viz_server.log'  C-m

#Starts state machine that controls all other components, required for active scene recognition.
tmux new-window -n 'state_machine'
tmux send-keys -t asr:state_machine 'script -c "roslaunch --wait asr_state_machine scene_exploration_sim.launch" -f '"${logFolder}"'/state_machine.log'  C-m

#Starts direct_search_manager that handles the direct search
tmux new-window -n 'direct_search_manager'
tmux send-keys -t asr:direct_search_manager 'script -c "roslaunch --wait asr_direct_search_manager direct_search_manager.launch" -f '"${logFolder}"'/direct_search_manager.log'  C-m

