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
#Script that is used to start active scene recognition in simulation of MILD robot.

# TODO new-window -d to not switch to new window, send-keys -t target-pane, split-window -t target-pane
# TODO rewrite as python script? because better/easier understandable api
# check for ros
source ~/.bashrc 
if ! hash roscd 2> /dev/null; then 
  echo "source ros in your bashrc"
  exit 1
fi

#Start tmux session if not already existing.
if ! { [ "$TERM" = "screen" ] && [ -n "$TMUX" ]; } then
  tmux new-session -d -s asr
  tmux send-keys 'roscd asr_resources_for_active_scene_recognition/launch/; ./start_modules_sim.sh' C-m
  tmux attach
  exit 1
fi

#So you know where roscore actually runs...
tmux rename-window 'roscore'
tmux send-keys 'roscore' C-m

#Starting of gazebo simulation of robot and its environment. Sets environment variables that is read by other components. To prevent breaking of tf tree. Please first start gazebo, then the rest.
tmux split-window
if [[ $DISPLAY ]]; then 
  tmux send-keys -t asr:roscore.1 "$pane" 'echo "Please shut down all components and execute following roslaunch before restarting the rest."; roslaunch --wait asr_mild_navigation gazebo_mild_manual_rearranged.launch' C-m
else
  tmux send-keys -t asr:roscore.1 'echo "Please shut down all components and execute following roslaunch before restarting the rest."; roslaunch --wait asr_mild_navigation gazebo_mild_manual_rearranged_headless.launch' C-m
fi

#Call this command once you started gazebo in preceeding pane.
tmux new-window -n 'post_gazebo_components' 

tmux send-keys -t asr:post_gazebo_components 'roscd asr_resources_for_active_scene_recognition; ./launch/helpers/start_modules_sim_post_gazebo_components.sh' C-m
#Start basic sensor and control components for MILD robot simulation as well as simulated object localization.
tmux send-keys -t asr:post_gazebo_components 'roscd asr_resources_for_active_scene_recognition; ./launch/helpers/start_modules_mock_robot.sh' C-m
#terminate window after everything is started
tmux send-keys -t asr:post_gazebo_components 'tmux kill-window -t post_gazebo_components' C-m

tmux new-window -n 'control'
# tmux send-keys 'roslaunch  --wait asr_resources_for_active_scene_recognition control.launch' C-m
tmux send-keys -t asr:control 'roscd asr_resources_for_active_scene_recognition; ./launch/control.sh' C-m

#Start rviz
tmux new-window -n 'rviz'
tmux send-keys -t asr:rviz 'roscd asr_resources_for_active_scene_recognition; roslaunch --wait launch/rviz_scene_exploration.launch' C-m
#tmux send-keys 'while true; do roscd asr_resources_for_active_scene_recognition; roslaunch launch/rviz_scene_exploration.launch; done' C-m
