#!/usr/bin/env python

'''
Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Karrenbauer Oliver, Marek Felix, Meissner Pascal, Stroh Daniel, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

#Manually draw frustum at current robot pose in RViz.
import roslib
import rospy
from asr_robot_model_services.msg import RobotStateMessage
from asr_robot_model_services.srv import CalculateCameraPose, GetPose
from next_best_view.srv import TriggerFrustumVisualization

def get_camera_pose_cpp():
    """
    Returns camera pose
    """
    rospy.wait_for_service('/asr_robot_model_services/GetCameraPose', timeout=5)
    pose = rospy.ServiceProxy('/asr_robot_model_services/GetCameraPose',GetPose)
    return pose().pose


def get_camera_frustum(camera_pose):
    try:
        rospy.wait_for_service('/nbv/trigger_frustum_visualization', timeout=5)
        get_frustum = rospy.ServiceProxy(
            '/nbv/trigger_frustum_visualization', TriggerFrustumVisualization)
        get_frustum(camera_pose)
    except (rospy.ServiceException, rospy.exceptions.ROSException), e:
        rospy.logwarn(str(e))


def main():
    camera_pose = get_camera_pose_cpp()
    get_camera_frustum(camera_pose)

if __name__ == "__main__":
    main()