#from control import *
import csv
import rospy
from next_best_view.srv import *
import imp
from configuration import *
import pickle
import numpy as np
from tmux import *
from configuration import *
from move import *
from general_ros import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
import copy
import math
import __builtin__

state_acquisition = imp.load_source("*", scene_exploration_path + "/src/states/state_acquisition.py")
nbvSettingsFilePath = next_best_view_path + "/rsc/next_best_view_settings_sim.yaml"

def restartAndWaitForNBV():
  restartPanes([nbvPane])
  waitForAllModules(True)

def setOptParameters():
  dataMap = getYaml(nbvSettingsFilePath)
  dataMap["minIterationSteps"] = 15
  dataMap["epsilon"] = 0.000000001
  dataMap["spaceSamplerId"] = 1
  dataMap["radius"] = 0.05
  dataMap["enableGA"] = False
  dataMap["cacheResults"] = False
  dataMap["sampleSizeUnitSphereSampler"] = 2000
  saveYaml(nbvSettingsFilePath, dataMap, True)

def setOptcacheParameters():
  dataMap = getYaml(nbvSettingsFilePath)
  dataMap["minIterationSteps"] = 15
  dataMap["epsilon"] = 0.000000001
  dataMap["spaceSamplerId"] = 1
  dataMap["radius"] = 0.05
  dataMap["enableGA"] = False
  dataMap["cacheResults"] = True
  dataMap["sampleSizeUnitSphereSampler"] = 2000
  saveYaml(nbvSettingsFilePath, dataMap, True)

def setOldParameters():
  dataMap = getYaml(nbvSettingsFilePath)
  dataMap["minIterationSteps"] = 1
  dataMap["epsilon"] = 0.01
  dataMap["spaceSamplerId"] = 1
  dataMap["radius"] = 0.2
  dataMap["enableGA"] = False
  dataMap["cacheResults"] = False
  dataMap["sampleSizeUnitSphereSampler"] = 384
  saveYaml(nbvSettingsFilePath, dataMap, True)

def setOldcacheParameters():
  dataMap = getYaml(nbvSettingsFilePath)
  dataMap["minIterationSteps"] = 1
  dataMap["epsilon"] = 0.01
  dataMap["spaceSamplerId"] = 1
  dataMap["radius"] = 0.2
  dataMap["enableGA"] = False
  dataMap["cacheResults"] = True
  dataMap["sampleSizeUnitSphereSampler"] = 384
  saveYaml(nbvSettingsFilePath, dataMap, True)

def setGAParameters():
  dataMap = getYaml(nbvSettingsFilePath)
  dataMap["minIterationSteps"] = 5
  dataMap["epsilon"] = 0.01
  dataMap["spaceSamplerId"] = 5
  dataMap["radius"] = 0.2
  dataMap["enableGA"] = True
  dataMap["cacheResults"] = True
  dataMap["sampleSizeUnitSphereSampler"] = 384
  saveYaml(nbvSettingsFilePath, dataMap, True)
  
def getOptimalNBV(logDir, idx, positionAndOrientation):
  bkp = getYaml(nbvSettingsFilePath)
  setOptimalParameters()
  restartAndWaitForNBV()
  setPositionAndOrientation(positionAndOrientation)
  setPCAndViewportsFromLog(logDir, idx)
  nbv = getNBV()
  saveYaml(nbvSettingsFilePath, bkp, True)
  restartAndWaitForNBV()
  return nbv

def getCurrentCameraPose():
  return state_acquisition.get_camera_pose_cpp()

def setRobotState():
    # try to get current robot config and set it in the nbv
    getRobotState = state_acquisition.GetRobotState()
    robotState = getRobotState.get_robot_state()

    rospy.wait_for_service('/nbv/set_init_robot_state')
    set_init_state = rospy.ServiceProxy('/nbv/set_init_robot_state', SetInitRobotState)
    set_init_state(robotState)

def getNBV():
  get_nbv = rospy.ServiceProxy('/nbv/next_best_view', GetNextBestView)
  setRobotState()
  current_camera_pose = getCurrentCameraPose()
  return get_nbv(current_camera_pose)

def nbvUpdateFromNBV(nbv):
  return nbvUpdate(getPoseFromNBV(nbv), nbv.object_type_name_list)

def nbvUpdate(pose, obj_types):
  nbv_update = rospy.ServiceProxy('/nbv/update_point_cloud', UpdatePointCloud)
  return nbv_update(pose, obj_types)

def getPC():
  get_pc = rospy.ServiceProxy("/nbv/get_point_cloud", GetAttributedPointCloud)
  return get_pc()

def getPCAndViewportsFromLog(logDir, idx):
  pointCloudFile = open(logDir + "/PointCloud/PointCloud_" + str(idx), "rb")
  viewportsFile = open(logDir + "/PointCloud/Viewports_" + str(idx), "rb")
  return (pickle.load(pointCloudFile), pickle.load(viewportsFile))

def setPCAndViewportsFromLog(logDir, idx):
  setPC(*getPCAndViewportsFromLog(logDir, idx))

def setPC(pointcloud, viewports):
  set_pc = rospy.ServiceProxy("/nbv/set_point_cloud", SetAttributedPointCloud)
  return set_pc(pointcloud, viewports)

def getNumberOfNBVs(logDir):
  fileContent = getFile(logDir + "/nbv_results.csv")
  if fileContent is None:
    return -1
  count = fileContent.count("utility:")
  return count

def getNBVs(logDir):
  try:
    fileName = logDir + "/nbv_results.csv"
    f = open(fileName)
    csvReader = csv.reader(f, delimiter = " ", quotechar = "|")
    nbvsTuples = list(csvReader)
    nbvs = map(lambda x: yaml.load(x[1]), nbvsTuples)
    return nbvs
  except IOError:
    print("file does not exist")
    return None

# camera pose
def getPoseFromNBV(nbv):
  if type(nbv) is not dict:
    point = copy.deepcopy(nbv.resulting_pose.position)
    orientation = copy.deepcopy(nbv.resulting_pose.orientation)
  else:
    pointDict = nbv["resulting_pose"]["position"]
    point = Point(pointDict["x"], pointDict["y"], pointDict["z"])
    orientationDict = nbv["resulting_pose"]["orientation"]
    orientation = Quaternion(orientationDict["x"], orientationDict["y"], orientationDict["z"], orientationDict["w"])
  return Pose(point, orientation)

# robot state
def getPositionAndOrientationFromNBV(nbv):
  if type(nbv) is dict:
    robotState = nbv["robot_state"]
    return ((robotState["x"], robotState["y"], math.degrees(robotState["rotation"])), (math.degrees(robotState["pan"]), math.degrees(robotState["tilt"])))
  else:
    robotState = nbv.robot_state
    return ((robotState.x, robotState.y, math.degrees(robotState.rotation)), (math.degrees(robotState.pan), math.degrees(robotState.tilt)))    

def getTotalDistance(nbvs):
  nbvPositions = map(lambda nbvResult: nbvResult["resulting_pose"]["position"], nbvs)
  nbvNPPos = map(lambda p: np.array([p["x"], p["y"], p["z"]]), nbvPositions)
  sum = 0
  for i in range(0, len(nbvNPPos) - 1):
    sum += np.linalg.norm(nbvNPPos[i] - nbvNPPos[i+1])
  return sum

def getRating(nbv):
  ratingNormalization = rospy.get_param("/nbv/mOmegaUtility") * rospy.get_param("/nbv/mOmegaPan") * \
    rospy.get_param("/nbv/mOmegaTilt") * rospy.get_param("/nbv/mOmegaRot") * \
    rospy.get_param("/nbv/mOmegaBase") * rospy.get_param("/nbv/mOmegaRecognizer")
  return (nbv["utility"] + nbv["inverse_costs"]) / ratingNormalization

def rateViewport(pose):
  if type(pose) is not list:
    pose = [pose]
  rate_viewports = rospy.ServiceProxy("/nbv/rate_viewports", RateViewports)
  return rate_viewports(getCurrentCameraPose(), pose, ["Cup", "PlateDeep"], False)

def getActualNBV(nbv):
  oldPos = (basePos, ptuPos)
  position, orientation = getPositionAndOrientationFromNBV(nbv)
  moveBase(*position)
  setPTU(*orientation)
  nbvCameraPose = getCurrentCameraPose()
  print(getCurrentCameraPose())
  print(str((basePos, ptuPos)))
  setPositionAndOrientation(oldPos)
  ratedMovedNbv = rateViewport(nbvCameraPose)
  return ratedMovedNbv

# this actually takes nbvs as parameters
pub = rospy.Publisher("/nbv/viewportsVisualization", MarkerArray, queue_size=10)

def showPoses(poses):
  if type(poses) is not list:
    poses = [poses]
  viewportMarkers = MarkerArray()
  i = 0
  for pose in poses:
    # straight vertical line
    newMarker = Marker()
    newMarker.color.r = 1.0
    newMarker.color.a = 1.0
    newMarker.scale.x = 0.05
    newMarker.scale.y = 0.05
    newMarker.scale.z = 0.05
    lowerPoint = copy.deepcopy(pose.position)
    lowerPoint.z = 0
    upperPoint = copy.deepcopy(pose.position)
    newMarker.points.append(lowerPoint)
    newMarker.points.append(upperPoint)# perfect = getNBVs(mayFolder)[0]
    newMarker.header.frame_id = "/map"
    newMarker.ns = "viewports"
    newMarker.id = 0 + i * 2
    newMarker.type = Marker.LINE_LIST
    newMarker.action = Marker.ADD
    viewportMarkers.markers.append(newMarker)
    # arrow
    newMarker = Marker()
    newMarker.color.r = 1.0
    newMarker.color.a = 1.0
    newMarker.scale.x = 0.5
    newMarker.scale.y = 0.05
    newMarker.scale.z = 0.05
    newMarker.pose.position = pose.position

    newMarker.pose.orientation = pose.orientation
    newMarker.header.frame_id = "/map"
    newMarker.ns = "viewports"
    newMarker.id = 1 + i * 2
    newMarker.type = Marker.ARROW
    newMarker.action = Marker.ADD
    viewportMarkers.markers.append(newMarker)
    i += 1
  
  pub.publish(viewportMarkers)

def showNBVs(nbvs):
  if type(nbvs) is not list:
    nbvs = [nbvs]
  poses = []
  for nbv in nbvs:
    poses.append(getPoseFromNBV(nbv))
  showPoses(poses)
