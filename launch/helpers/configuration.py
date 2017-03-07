import re
import os
import xml.etree.ElementTree
import yaml
import stat
from move import *
from general_ros import *
from tmux import restartPanes
from math import *

def getFile(fileLoc):
  try:
    f = file(fileLoc, "r")
    content = f.read()
    f.close()
  except IOError:
    print("invalid file")
    return None
  return content

# log dir
homeDir = os.path.expanduser("~")
logDir = homeDir + "/log/"
def getLogFolders():
  folders = filter(lambda x: os.path.isdir(logDir + x), os.listdir(logDir))
  # create (folder, time) tuples
  foldersTime = map(lambda folder: (os.stat(logDir + folder)[stat.ST_MTIME], logDir + folder), folders)
  foldersTime.sort()
  # folder contains now sorted log directory by creation time
  logFolders = map(lambda x: x[1], foldersTime)
  return logFolders

def latestLogFolder():
  logFolders = getLogFolders()
  return logFolders[-1]

def rmFile(filename):
  try:
    os.remove(filename)
  except:
    pass

def reduceLogSize(logFolder):
  rmFile(logFolder + "/OBJ.bag.active")
  rmFile(logFolder + "/OBJ.bag")
  rmFile(logFolder + "/ism.log")
  rmFile(logFolder + "/world_model.log")

logFolders = getLogFolders()

def getLogSubFolders(folder):
  results = []
  for subfolder in os.listdir(folder):
    if os.path.isdir(folder + "/" + subfolder):
      if isLogFolder(folder + "/" + subfolder):
        results.append(folder + "/" + subfolder)
      else:
        results.extend(getLogSubFolders(folder + "/" + subfolder))
  return results

def isLogFolder(folder):
  return os.path.isfile(folder + "/state_machine.log")

# READ: this is pretty neat, but it removes comments, so be careful
def setConfig(cfg, restart = True):
  writeWorldLaunchFile(world_model_path + "/launch/world_model.launch", cfg)
  writeWorldLaunchFile(world_model_path + "/launch/whole_test_system.launch", cfg)
  writeDatabase(recognizer_prediction_ism_path + "/launch/helpers/scene_recognition.yaml", cfg)
  writeFakeRecognizer(fake_object_recognition_path + "/config/params.yaml", cfg)
  if restart:
    print("restarting...")
    restartConfigPanes()

# to write config to files
def writeWorldLaunchFile(file, cfg):
  et = xml.etree.ElementTree.parse(file)
  rosparams =  et.findall(".//rosparam")
  rosparams = filter(lambda x : "world_description" in x.attrib["file"], rosparams)
  assert(len(rosparams) == 1)
  rosparams[0].attrib["file"] = cfg["world_description"]
  et.write(file)

def writeDatabase(file, cfg):
  dataMap = getYaml(file)
  dataMap["dbfilename"] = cfg["database"]
  saveYaml(file, dataMap)

def writeFakeRecognizer(file, cfg):
  dataMap = getYaml(file)
  dataMap["config_file_path"] = cfg["fake_recognizer_objs"]
  saveYaml(file, dataMap)

# yaml
def getYaml(file):
  f = open(file)
  dataMap = yaml.load(f)
  f.close()
  return dataMap

def saveYaml(file, dataMap, flow_style=False):
  f = open(file, "w")
  if flow_style is True:
    yaml.dump(dataMap, f)
  else:
    yaml.dump(dataMap, f, default_flow_style=False)
  f.close()

roscorePane = (0, 0)
gazeboPane = (0, 1)
controlPane = (2, 0)
rvizPane = (3, 0)
ismPane = (4, 0)
nbvPane = (5, 0)
worldModelPane = (5, 1)
vizServerPane = (6, 0)
smPane = (7, 0)
initialSearchmanagerPane = (8, 0)
movePane = (9, 0)
ptuDriverPane = (10, 0)
ptuControllerPane = (10, 1)
ptuControllerCliPane = (10, 2)
ptuRefreshFrustumPane = (10, 3)
fakeRecognizerPane = (11, 0)
allPanes = [
  ismPane,
  nbvPane,
  worldModelPane,
  vizServerPane,
  smPane,
  initialSearchmanagerPane,
  movePane,
  ptuDriverPane,
  ptuControllerPane,
  fakeRecognizerPane
]

configPanes = [
  ismPane,
  worldModelPane,
  smPane,
  fakeRecognizerPane
]

orientations = [
  (0, -35)
]

positions = [
  # 
  [
    (-1.1, -0.34, 110),
    (-1.4, -0.22, 90),
    (-0.6, -0.13, 135),
    (-0.48, -0.09, 140),
    (-0.44, 0.27, 165)
  ],
  #
  [
    (0.38, -1.81, 160),
    (0.37, -1.48, 180),
    (0.34, -1.13, 200),
    (0.27, -0.87, 215),
    (0.086, -0.64, 230),
    (-0.11, -0.5, 245),
    (-0.41, -0.53, 260),
    (-0.685, -0.54, 275),
    (-0.956, -0.56, 290),
    (-1.183, -0.56, 300)
  ],
  #
  [
    (2.147, -1.9, 110),
    (1.78, -1.9, 90),
    (1.39, -1.87, 65),
    (1.06, -1.67, 45),
    (0.93, -1.37, 25),
    (0.9, -1.04, 5),
    (0.9, -0.77, 0),
    (0.91, -0.45, -20),
    (0.96, -0.185, -30),
    (1.09, 0.02, -40),
    (1.31, 0.16, -60)
  ],
  #
  [
    (1.42, 0.196, 0),
    (1.41, 0.43, -30),
    (1.288, 0, 0)
  ],
  #
  [
    (-0.44, 1.9, 90),
    (-0.18, 1.88, 90),
    (-0.11, 2.02, 90)
  ]
]

# setConfig(cfg_story1)
# configuration for story1
# world_description is a setting in world_model/launch launch files
# database contains the scene and is defined in asr_recognizer_prediction_ism/rsc/scene_recognition.yaml
# fake_recognizer_objs contains detectable objects and is defiend in asr_fake_object_recognition/config/params.yaml
cfg_story1_missing_knife = {
  "world_description" : "$(find asr_world_model)/rsc/world_descriptions/world_description.yaml",
  "database" : resources_for_active_scene_recognition_path + "/scene_recordings/story_1.sqlite",
  "fake_recognizer_objs" : "./config/story_1.xml",
  "startPositions" : [(positions[0][0], orientations[0])]
}

cfg_story1_schrank = {
  "world_description" : "$(find asr_world_model)/rsc/world_descriptions/world_description.yaml",
  "database" : resources_for_active_scene_recognition_path + "/scene_recordings/story_1.sqlite",
  "fake_recognizer_objs" : "./config/story_1_schrank.xml",
  "startPositions" : [(positions[4][0], orientations[0])]
}

# configuration for story2
cfg_story2 = {
  "world_description" : "$(find asr_world_model)/rsc/world_descriptions/world_description.yaml",
  "database" : resources_for_active_scene_recognition_path + "/scene_recordings/story_1.sqlite", # TODO: not there
  "fake_recognizer_objs" : "./config/story_2_tisch.xml" # TODO: might vary from story 2 mission
}

# configuration for 25th may
cfg_may25 = {
  "world_description" : "$(find asr_world_model)/rsc/world_descriptions/world_description_25th_may.yaml",
  "database" : resources_for_active_scene_recognition_path + "/scene_recordings/scene_25th_may.sqlite",
  "fake_recognizer_objs" : "./config/scene_25th_may.xml",
  "startPositions" : [(positions[2][5], orientations[0])]
}

cfgs = [
  cfg_story1_missing_knife,
  cfg_story1_schrank,
  cfg_story2,
  cfg_may25
]

def getConfig(folder):
  cfg = {}
  highest_idx = -1
  for file in os.listdir(folder):
    if os.path.isfile(folder + "/" + file):
      if file.endswith("sqlite"):
        cfg["database"] = os.path.abspath(folder + "/" + file)
      constellation_indices = re.findall("recognition_(.*)_constellation.*xml", file)
      constellation_idx = int(constellation_indices[0]) if len(constellation_indices) > 0 else -1
      if constellation_idx > highest_idx:
        highest_idx = constellation_idx
  cfg["world_description"] = "$(find world_model)/rsc/world_descriptions/world_description.yaml"
  cfg["fake_recognizer_objs"] = os.path.abspath(folder + "/" + "recognition_" + str(highest_idx) + "_constellation_0.xml")
  return cfg

def getStartPosition(folder):
  f = open(folder + "/state_machine.log", "r")
  content = f.read()
  result = re.findall("Initial robot state:\r$\n^(.*\r$\n^.*\r$\n^.*\r$\n^.*\r$\n^.*)\r", content, re.M)
  if len(result) == 0:
    return None
  robot_state_str = result[0]
  rows = robot_state_str.split("\r\n")
  robot_state_map = {}
  for row in rows:
    key = row.split(":")[0].strip()
    value = float(row.split(":")[1].strip())
    robot_state_map[key] = value
  pos = (robot_state_map["x"], robot_state_map["y"], degrees(robot_state_map["rotation"]))
  orientation = (robot_state_map["pan"], robot_state_map["tilt"])
  return (pos, orientation)

def restartConfigPanes():
  restartPanes(configPanes)

def getCurrentConfig():
  dataMap = getYaml(fake_object_recognition_path + "/config/params.yaml")
  for cfg in cfgs:
    if cfg["fake_recognizer_objs"] == dataMap["config_file_path"]:
      return cfg

def setDefaultPos():
  startPos = getCurrentConfig()["startPositions"][0]
  setPositionAndOrientation(startPos)