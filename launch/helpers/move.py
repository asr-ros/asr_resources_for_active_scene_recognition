from general_ros import *
from configuration import *
from asr_msgs.msg import AsrObject, AsrViewport
from asr_flir_ptu_controller.msg import PTUMovementGoal, PTUMovementAction, PTUMovementActionResult
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseActionResult
from geometry_msgs.msg import (Pose, PoseWithCovariance, PoseWithCovarianceStamped)
from asr_robot_model_services.srv import GetPose
from actionlib import *
from actionlib.msg import *
import subprocess
import math
import tf
import threading
import __builtin__

# cur ptu position to move ptu by an offset
__builtin__.ptuPos = (0, 0) # (pan, tilt)
# callback to get cur ptu position
def ptuMoved(result):
  if len(result.result.end_joint.position) == 2:
    __builtin__.ptuPos = (result.result.end_joint.position[0], result.result.end_joint.position[1])
  else:
    print("invalid ptu result")
rospy.Subscriber("/ptu_controller_actionlib/result", PTUMovementActionResult, ptuMoved)

# to get base position
__builtin__.basePos = (0, 0, 0)
def baseMoved(idc):
  thread = threading.Thread(target=getBasePos)
  thread.start()
  thread.join()
def getBasePos():
  tfListener = tf.TransformListener()
  t = rospy.Time()
  tfListener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(4.0))
  pos, quat = tfListener.lookupTransform("/map", "/base_link", t)
  orientationAngle = math.degrees(math.asin(quat[2]) * 2)
  __builtin__.basePos = (pos[0], pos[1], orientationAngle)
rospy.Subscriber("/move_base/result", MoveBaseActionResult, baseMoved)
rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, baseMoved)

# to set base
set_base_client = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

# ptu
def movePTU(pan, tilt):
  newPTUPos = (__builtin__.ptuPos[0] + pan, __builtin__.ptuPos[1] + tilt)
  setPTU(*newPTUPos)

def setPTU(pan, tilt):
  ptu_client = actionlib.SimpleActionClient("ptu_controller_actionlib", PTUMovementAction)
  if not ptu_client.wait_for_server():
    print("Could not connect to ptu action server")
  ptu_goal = PTUMovementGoal()
  ptu_goal.target_joint.header.seq = 0
  ptu_goal.target_joint.name = ["pan", "tilt"]
  ptu_goal.target_joint.velocity = [0, 0]
  ptu_goal.target_joint.position = [float(pan), float(tilt)]
  result = ptu_client.send_goal_and_wait(ptu_goal)
  if result != 3:
    print("setPTU failed")
  # visualization
  #waitForService("/asr_robot_model_services/GetCameraPose")
  waitForService("/nbv/trigger_frustum_visualization", True)
  subprocess.call("python " + resources_for_active_scene_recognition_path + "/src/getFrustum.py", shell=True)
  return ptu_client.get_result()

# base
def moveBase(x, y, rotation):
  move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
  if not move_base_client.wait_for_server():
    print("Could not connect to move base action server")
  move_base_goal = MoveBaseGoal()
  move_base_goal.target_pose.header.frame_id = "/map"
  move_base_goal.target_pose.pose.position.x = x
  move_base_goal.target_pose.pose.position.y = y
  move_base_goal.target_pose.pose.orientation.w = math.cos(math.radians(rotation) / 2.0)
  move_base_goal.target_pose.pose.orientation.z = math.sin(math.radians(rotation) / 2.0)
  result = move_base_client.send_goal_and_wait(move_base_goal)
  if result != 3:
    print("moveBase failed")
  return move_base_client.get_result()

def setBase(x, y, rotation):
  set_base_goal = PoseWithCovarianceStamped()
  set_base_goal.header.frame_id = "map"
  set_base_goal.pose.pose.position.x = float(x)
  set_base_goal.pose.pose.position.y = float(y)
  set_base_goal.pose.pose.orientation.w = math.cos(math.radians(float(rotation)) / 2.0)
  set_base_goal.pose.pose.orientation.z = math.sin(math.radians(float(rotation)) / 2.0)
  set_base_client.publish(set_base_goal)

def setPositionAndOrientation(positionAndOrientation):
  setBase(*positionAndOrientation[0])
  setPTU(*positionAndOrientation[1])
