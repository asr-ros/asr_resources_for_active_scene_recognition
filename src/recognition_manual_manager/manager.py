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

import roslib
import rospy
import recognition_for_grasping.srv
from asr_msgs.msg import AsrObject
from asr_world_model.srv import PushFoundObject, PushFoundObjectList
import asr_aruco_marker_recognition.srv 
import asr_fake_object_recognition.srv
from asr_world_model.srv import GetRecognizerName
import asr_descriptor_surface_based_recognition.srv 


class ObjectDetectorsManager:

    def __init__(self):
        pass

    def start_recognizers_descriptor(self, search_objects):
        try:
            rospy.wait_for_service(
                    '/asr_descriptor_surface_based_recognition/get_recognizer',
                    timeout=3)
            descriptor_recognizer = rospy.ServiceProxy(
                    '/asr_descriptor_surface_based_recognition/get_recognizer',
                    asr_descriptor_surface_based_recognition.srv.GetRecognizer)
        except (rospy.exceptions.ROSException, rospy.ServiceException) as e:
                rospy.logwarn("Service error with descriptor recognizer")
                return 'aborted'
        descriptor_recognizer(search_objects, 1, False)
        rospy.loginfo('Detection started for ' + str(search_objects))
        return 'succeeded'

    def start_markers(self):
        try:
            rospy.wait_for_service(
                    '/asr_aruco_marker_recognition/get_recognizer',
                    timeout=3)
            marker_recognizer = rospy.ServiceProxy(
                    '/asr_aruco_marker_recognition/get_recognizer',
                    asr_aruco_marker_recognition.srv.GetRecognizer)
            marker_recognizer()
        except (rospy.exceptions.ROSException, rospy.ServiceException) as e:
            rospy.logwarn("Service error with marker recognizer")
            return 'aborted'
        rospy.loginfo('Marker recognition started.')
        return 'succeeded'

    def start_recognizers_rfg(self, search_objects, type):
        recognizer = None

        # initialize recognition manager
        try:
            rospy.wait_for_service('/recognition_manager/get_recognizer',
                                   timeout=2)
            recognizer = rospy.ServiceProxy(
                    '/recognition_manager/get_recognizer',
                    recognition_for_grasping.srv.GetRecognizer)
        except (rospy.exceptions.ROSException, rospy.ServiceException) as e:
            rospy.logwarn("Service error with recognition manager")
            return 'aborted'

        # start the recognizers
        # marker recognizer does not have to be started, it's looking for all
        # object
        recognizer(str(search_objects), str(type), False)
        rospy.loginfo('Detection started for ' + str(search_objects))
        return 'succeeded'

    def stop_recognizers_descriptor(self, search_objects):
        release_descriptor_recognizer = None
        try:
            release_descriptor_recognizer = rospy.ServiceProxy(
                    '/asr_descriptor_surface_based_recognition/release_recognizer',
                    asr_descriptor_surface_based_recognition.srv.ReleaseRecognizer)
        except rospy.ServiceException, e:
            rospy.logwarn("Error calling the release recognizer services for descriptor recognition.")
            return 'aborted'
        release_descriptor_recognizer(search_objects)
        rospy.loginfo(str(search_objects)+" released.")
        return 'succeeded'

    def stop_all_recognizers_descriptor(self):
        clear_all_descriptor_recognizers = None
        try:
            clear_all_descriptor_recognizers = rospy.ServiceProxy(
                    '/asr_descriptor_surface_based_recognition/clear_all_recognizers',
                    asr_descriptor_surface_based_recognition.srv.ClearAllRecognizers)
        except rospy.ServiceException, e:
            rospy.logwarn("Error calling the clear all recognizers service for descriptor recognition.")
            return 'aborted'
        clear_all_descriptor_recognizers()
        rospy.loginfo("All descriptor recognizers released.")
        return 'succeeded'

    def stop_recognizers_rfg(self, search_objects, type):
        release_recognizer = None
        try:
            release_recognizer = rospy.ServiceProxy(
                    '/recognition_manager/release_recognizer',
                    recognition_for_grasping.srv.ReleaseRecognizer)
        except rospy.ServiceException, e:
            rospy.logwarn("Error calling the release recognizer services for recognition manager.")
            return 'aborted'
        # release recognizer
        release_recognizer(str(search_objects), str(type), "/stereo/objects")
        rospy.loginfo(str(search_objects)+" released.")
        return 'succeeded'

    def stop_all_recognizers_rfg(self):
        clear_all_recognizers = None
        try:
            clear_all_recognizers = rospy.ServiceProxy(
                    '/recognition_manager/clear_all_recognizers',
                    recognition_for_grasping.srv.ClearAllRecognizers)
        except rospy.ServiceException, e:
            rospy.logwarn("Error calling the clear all recognizers service for recognition manager.")
            return 'aborted'
        # clear all recognizers
        clear_all_recognizers("/stereo/objects")
        rospy.loginfo("All rfg recognizers released.")
        return 'succeeded'

    def stop_markers(self):
        try:
            release_marker_recognizer = rospy.ServiceProxy(
                    '/asr_aruco_marker_recognition/release_recognizer',
                    asr_aruco_marker_recognition.srv.ReleaseRecognizer)
        except rospy.ServiceException, e:
            rospy.logwarn("Error calling the release recognizer services for marker recognition.")
            return 'aborted'
        release_marker_recognizer()
        rospy.loginfo('Marker recognition stopped.')
        return 'succeeded'

def main():
    rospy.init_node('recognition_manual_manager')
    manager = ObjectDetectorsManager()
    while not rospy.is_shutdown():
        command_string = raw_input('Enter a valid command, f.e start,Smacks,textured :')
        command_array = command_string.split(',')

        if command_array[0] == 'start':
            if command_array[1] == 'markers':
                manager.start_markers()
            elif command_array[2] == 'descriptor':
                manager.start_recognizers_descriptor(command_array[1])
            elif command_array[2] in 'textured segmentable':
                manager.start_recognizers_rfg(command_array[1],command_array[2])
            else:
                rospy.loginfo('invalid command')

        elif command_array[0] == 'stop':
            if command_array[1] == 'markers':
                manager.stop_markers()
            elif command_array[1] == 'all':
                manager.stop_all_recognizers_rfg()
                manager.stop_all_recognizers_descriptor()
                manager.stop_markers()
            elif command_array[2] == 'descriptor':
                manager.stop_recognizers_descriptor(command_array[1])               
            elif command_array[2] in 'textured segmentable':
                manager.stop_recognizers_rfg(command_array[1],command_array[2])
            else:
                rospy.loginfo('invalid command')

        else:
            rospy.loginfo('invalid command')

    rospy.spin()

if __name__ == '__main__':
    main()
