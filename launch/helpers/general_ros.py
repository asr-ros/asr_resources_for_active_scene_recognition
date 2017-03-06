import rospy
import rospkg

# to wait until application is fully started
def waitForService(serviceName, silence = False):
  if not silence:
    print("waiting for " + serviceName)
  rospy.wait_for_service(serviceName, timeout=60)
  # print("finished waiting for " + serviceName)

def waitForAllModules(silence = False):
  try:
    waitForService("/rp_ism_node/find_scenes", silence)
    waitForService("/visualization/clear_all_models", silence)
    waitForService("/robot_state_publisher/get_loggers", silence)
    waitForService("/asr_flir_ptu_driver/get_range", silence)
    waitForService("/ptu_manual_controller_script/get_loggers", silence)
    #waitForService("/asr_robot_model_services/GetRobotPose")
    waitForService("/asr_object_database/recognizer_list", silence)
    waitForService("/nbv/set_point_cloud", silence)
    waitForService("/move_base/make_plan", silence)
    waitForService("/asr_mild_base_fake_driving/get_loggers", silence)
    waitForService("/map_server/get_loggers", silence)
    waitForService("/fake_object_recognition/get_all_recognizers", silence)
    waitForService("/env/asr_world_model/get_viewport_list", silence)
    waitForService("/PTUController/get_loggers", silence)
    return True
  except rospy.ROSException, e:
    return False

# get an instance of RosPack with the default search paths, see http://wiki.ros.org/Packages#Python
rospack = rospkg.RosPack()

# ros package paths
scene_exploration_path = rospack.get_path("asr_state_machine")
resources_for_active_scene_recognition_path = rospack.get_path("asr_resources_for_active_scene_recognition")
world_model_path = rospack.get_path("asr_world_model")
recognizer_prediction_ism_path = rospack.get_path("asr_recognizer_prediction_ism")
fake_object_recognition_path = rospack.get_path("fake_object_recognition")
next_best_view_path = rospack.get_path("asr_next_best_view")