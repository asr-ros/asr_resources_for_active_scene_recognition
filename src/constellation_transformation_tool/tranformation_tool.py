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

import sys
import rospy
import roslib
import subprocess, os, signal
import math
import rospkg
import datetime
import tf
import numpy

from geometry_msgs.msg import Point,Quaternion,Pose
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def multiply_matrix(matrix_1,matrix_2):

    result_matrix = []
    if len(matrix_1[0]) != len(matrix_2):
        print "Invalid Matrix"
        return False
    for i in range(len(matrix_1)):
        line = []
        for j in range(len(matrix_2[0])):
            element = 0
            for k in range(len(matrix_1[0])):
                element = element + matrix_1[i][k] * matrix_2[k][j]
            line.append(element)
        result_matrix.append(line)
    return result_matrix

def matrix_vector_mult(matrix,vector):
    result_vect = []
    for mat in matrix:
        result_vect.append(mat[0]*vector[0]+mat[1]*vector[1]+mat[2]*vector[2])
    return result_vect


def get_rotation_matrix(angle_x, angle_y, angle_z):
    rotation_matrix_x = []
    rotation_matrix_x.append([1,0,0])
    rotation_matrix_x.append([0,math.cos(angle_x),-math.sin(angle_x)])
    rotation_matrix_x.append([0,math.sin(angle_x),math.cos(angle_x)])

    rotation_matrix_y = []
    rotation_matrix_y.append([math.cos(angle_y),0,math.sin(angle_y)])
    rotation_matrix_y.append([0,1,0])
    rotation_matrix_y.append([-math.sin(angle_y),0,math.cos(angle_y)])

    rotation_matrix_z = []
    rotation_matrix_z.append([math.cos(angle_z),-math.sin(angle_z),0])
    rotation_matrix_z.append([math.sin(angle_z),math.cos(angle_z),0])
    rotation_matrix_z.append([0,0,1])

    return multiply_matrix(rotation_matrix_x,multiply_matrix(rotation_matrix_y,rotation_matrix_z))

def getCubeMarker(pose,k,n_s,r,g,b,orientation):
    """
    returns a sphere marker
    """
    color = ColorRGBA(*[r,g,b,1])
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.get_rostime()
    marker.ns = n_s
    marker.id = k
    marker.type = 1
    marker.action = 0
    marker.pose.position = pose
    marker.pose.orientation = orientation
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color = color
    marker.lifetime = rospy.Duration()
    return marker

def getMeshMarker(pose,k,n_s,orientation,meshpath):
    """
    returns a sphere marker
    """
    color = ColorRGBA(*[0,0,0,0])
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.get_rostime()
    marker.ns = n_s
    marker.id = k
    marker.type = 10
    marker.action = 0
    marker.pose.position = pose
    marker.pose.orientation = orientation
    marker.scale.x = 0.001
    marker.scale.y = 0.001
    marker.scale.z = 0.001
    marker.color = color
    marker.mesh_resource = meshpath
    marker.mesh_use_embedded_materials = True
    marker.lifetime = rospy.Duration()
    return marker

def getMarkerArrow(point,k,n_s,r,g,b,orientation):
    """
    returns an arrow marker
    """
    marker = Marker()
    color = ColorRGBA(*[r,g,b,1])
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.get_rostime()
    marker.ns = n_s
    marker.id = k
    marker.type = 0
    marker.action = 0
    marker.pose.position=point
    marker.pose.orientation = orientation
    marker.scale.x = 0.2
    marker.scale.y = 0.02
    marker.scale.z = 0.02
    marker.color = color
    return marker

if __name__ == "__main__":

    rospy.init_node('tranformation_tool')
    while not rospy.is_shutdown():
        angle_x = 0*math.pi/180
        angle_y = 0*math.pi/180
        angle_z = -150*math.pi/180

        translation = [-0.2,-0.6,0]

        point_constellation = []
        pose1 = Pose()
        pose1.position = Point(*[-0.0831926,0.787348,0.705691])
        pose1.orientation = Quaternion(*[-0.569348,-0.42073,0.411762,0.57383])
        path1 = 'package://asr_object_database/rsc/databases/textured_objects/Knaeckebrot/Knaeckebrot.dae'
        object1 = [pose1,path1]
        pose2 = Pose()
        pose2.position = Point(*[-0.58582,0.495014,0.786433])
        pose2.orientation = Quaternion(*[-0.579757,0.402269,-0.378432,0.599042])
        path2 = 'package://asr_object_database/rsc/databases/textured_objects/Smacks/Smacks.dae'
        object2 = [pose2,path2]
        pose3 = Pose()
        pose3.position = Point(*[-0.188009,0.682787,0.74663])
        pose3.orientation = Quaternion(*[-0.600667,-0.393664,0.370855,0.588808])
        path3 = 'package://asr_object_database/rsc/databases/textured_objects/VitalisSchoko/VitalisSchoko.dae'
        object3 = [pose3,path3]
        pose4 = Pose()
        pose4.position = Point(*[-0.399521,0.648461,0.687763])
        pose4.orientation = Quaternion(*[-0.708298,-0.0518195,0.0561499,0.701766])
        path4 = 'package://asr_object_database/rsc/databases/textured_objects/CoffeeBox/CoffeeBox.dae'
        object4 = [pose4,path4]


        point_constellation.append(object1)
        point_constellation.append(object2)
        point_constellation.append(object3)
        point_constellation.append(object4)
        new_constellation = []
        id = 0

        publisher = rospy.Publisher("tranformation_tool", Marker, queue_size=10)

        for poses in point_constellation:
            marker = Marker()
            marker = getMeshMarker(poses[0].position,id,'transformation_orig',poses[0].orientation,poses[1])
            id += 1
            publisher.publish(marker)

            marker = getMarkerArrow(poses[0].position,id,'transformation_orig',1,0,0,poses[0].orientation)
            id += 1
            publisher.publish(marker)


        for poses in point_constellation:


            base_vec = [poses[0].position.x,poses[0].position.y,poses[0].position.z]
            new_base_vec = matrix_vector_mult(get_rotation_matrix(angle_x, angle_y, angle_z),base_vec)
            new_position = Point(*[new_base_vec[0],new_base_vec[1],new_base_vec[2]])

            new_position.x += translation[0]
            new_position.y += translation[1]
            new_position.z += translation[2]

            q = numpy.array([
            poses[0].orientation.x,
            poses[0].orientation.y,
            poses[0].orientation.z,
            poses[0].orientation.w])
            euler_vec = tf.transformations.euler_from_quaternion(q)
            euler_vec = list(euler_vec)
            euler_vec[0] += angle_x
            euler_vec[1] += angle_y
            euler_vec[2] += angle_z
            new_quat = tf.transformations.quaternion_from_euler(euler_vec[0],euler_vec[1],euler_vec[2])

            new_pose = Pose()
            new_pose.position = new_position
            new_pose.orientation = Quaternion(*new_quat)
            object = [new_pose,poses[1]]
            new_constellation.append(object)

        for poses in new_constellation:
            marker2 = Marker()
            marker2 = getMeshMarker(poses[0].position,id,'transformation_new',poses[0].orientation,poses[1])
            publisher.publish(marker2)
            id += 1

            marker2 = getMarkerArrow(poses[0].position,id,'transformation_new',0,1,0,poses[0].orientation)
            id += 1
            publisher.publish(marker2)
            rospy.loginfo('Object: ' +poses[1])
            rospy.loginfo('posestring: ' + str(poses[0].position.x) +','+ str(poses[0].position.y) +','+ str(poses[0].position.z) +','
                          + str(poses[0].orientation.w) +','+ str(poses[0].orientation.x) +','+ str(poses[0].orientation.y) +','+ str(poses[0].orientation.z))


    rospy.spin()




