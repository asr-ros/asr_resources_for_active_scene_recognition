#!/usr/bin/python2

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

import os
from time import sleep
import rospy
import tf
import re
from tmux import *
from string_util import *
import subprocess
import rlcompleter
import readline
import time
import imp
import time
import psutil
import shutil
from nbv import *
from general_ros import *
from configuration import *
from move import *
from personal_stuff import *
from asr_world_model.srv import GetMissingObjectList
from asr_msgs.msg import AsrObject

# tab completion
readline.parse_and_bind("tab: complete")

# ros node
waitForAllModules()
rospy.init_node("general_configuration", anonymous = True)

# import stuff from state machine
ObjectDetectorsManager = imp.load_source("ObjectDetectorsManager", scene_exploration_path + "/src/states/object_detectors_manager.py").ObjectDetectorsManager()

# obj detection
global detectedObjects
detectedObjects = {}
def detection_callback(data):
  global detectedObjects
  if data.type not in detectedObjects:
    detectedObjects[data.type] = 1
  else:
    detectedObjects[data.type] += 1

def startRecognition(recognizers = [], waitTime = 2):
  global detectedObjects
  detectedObjects = {}
  if type(recognizers) is not list:
    recognizers = [recognizers]
  if len(recognizers) == 0:
    rospy.wait_for_service("/env/asr_world_model/get_missing_object_list", timeout=5)
    get_missing_object_list = rospy.ServiceProxy("/env/asr_world_model/get_missing_object_list", GetMissingObjectList)
    missing_pbd_typeAndId = get_missing_object_list().missingObjects
    print(missing_pbd_typeAndId)
    for obj in missing_pbd_typeAndId:
      recognizers.append(str(obj.type))
  ObjectDetectorsManager.start_recognizers(recognizers)
  future = time.time() + waitTime
  sub = rospy.Subscriber("/stereo/objects", AsrObject, detection_callback)
  while (time.time() < future):
      pass
  # unregister, otherwise the callbacks are still called
  sub.unregister()
  ObjectDetectorsManager.stop_recognizers(recognizers)
  return detectedObjects

# tmux
def tmuxStart():
  subprocess.call("tmux new-session -d", shell=True)
  subprocess.call("tmux send-keys \"roscd asr_resources_for_active_scene_recognition/launch/\" C-m", shell=True)
  subprocess.call("tmux send-keys \"./start_modules_sim.sh\" C-m", shell=True)

def tmuxRestart():
  tmuxKill()
  tmuxStart()

def terminateAllModules():
  # since tmuxkill does not always terminate everything
  processNames = ["gzserver", "roslaunch", "asr_world_model", "rosout", "visualization", "rp_ism_node", "roscore", "move_base", "asr_mild_base_fake_driving", "state_publisher", "grid_action_server", "asr_object_database", "asr_fake_object_recognition", "asr_flir_ptu_driver", "PTUController", "map_server", "asr_next_best_view", "next_best_view_robot_model", "rosbag", "record", "rviz", "static_transform_publisher", "rosmaster", "gdb"]
  for proc in psutil.process_iter():
    if proc.name in processNames or ("python" in proc.name and proc.pid != os.getpid()):
      print("terminating " + proc.name)
      proc.kill()

# sm
def restartSM():
  restartPanes(smPane)

def stopSM():
  terminatePanes(smPane)
  
def isSMFinished():
  result = getSMResult()
  return result != False
  # return "State machine terminating" in getLastLinesFrom(getTmuxOut("state_machine", 0), -2)

def waitSMFinished():
  while not isSMFinished():
    time.sleep(1)

def getLogSMResults(logdir):
  if isLogFolder(logdir):
    content = getFile(logdir + "/state_machine.log")
    return str(getSMResultFromFileContent(content))

def startSM():
  selectWindow("state_machine")
  while "indirect_search" not in getLastLinesFrom(getTmuxOut("state_machine", 0), -2):
    time.sleep(1)
  subprocess.call("tmux send-keys C-m", shell=True)

def getSMResultFromFileContent(smLog):
  result = re.findall(".*State machine terminating '.*(?:SEARCH|SCENE_RECOGNITION).*':'(.*)':'.*", smLog)
  return result[0] if type(result) == list and len(result) > 0 else False

def getSMResult():
  result = re.findall(".*State machine terminating '.*(?:SEARCH|SCENE_RECOGNITION).*':'(.*)':'.*", getFile(logDir + "/state_machine.log"))
  return result[0] if type(result) == list and len(result) > 0 else False

def startAll(basePos, ptuPos):
  tmuxRestart()
  if not waitForAllModules():
    tmuxKill()
    return False
  setBase(*basePos)
  setPTU(*ptuPos)
  startSM()
  return True

def startAllDefault():
  startAll(positions[0][0], orientations[0])
  
def start():
  waitForAllModules(True)
  setDefaultPos()
  startSM()

def restart():
  restartPanes(allPanes)
  waitForAllModules()
  setDefaultPos()
