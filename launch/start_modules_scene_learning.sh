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
#Script that is used to start learning of scene models.

#Start tmux session if not already existing.
if ! { [ "$TERM" = "screen" ] && [ -n "$TMUX" ]; } then
  tmux new-session -d -s asr
  tmux send-keys 'roscd asr_resources_for_active_scene_recognition/launch/; ./start_modules_scene_learning.sh' C-m
  tmux attach
  exit 1
fi

#So you know where roscore actually runs...
tmux rename-window 'roscore'
tmux send-keys 'roscore' C-m

#Start basic sensor and control components.
sh helpers/start_modules_robot.sh

#Starts scene recorder and interface for service calls to object localizers
tmux new-window -n 'recorder'
tmux send-keys -t asr:recorder.0 'roslaunch --wait ism recorder.launch' C-m
tmux split-window -t asr:recorder
tmux send-keys -t asr:recorder.1 'roscd asr_resources_for_active_scene_recognition/src/recognition_manual_manager/;python manager.py' C-m 
