#! /usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##

import rospy
import moveit_commander
import rospkg
import actionlib
from bark_msgs.msg import BoundingBoxDetectionAction, BoundingBoxDetectionGoal
from cv_bridge import CvBridge
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from sensor_msgs.msg import JointState

import sys
import copy
import os
from math import pi
import cv2

class BarkSlidebot:
    '''
    - mozgás a fotó pózba
    - fotó készítés
    - fotó továbbküldés a bbox detectionnak
    - bbox fölé mozgás
    - orientáció becslés
    - üveglemez detektálás
    - megfogási póz visszavetítés
    - megfogás
    '''
    def __init__(self, group_name="manipulator"):
        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        self.robot = moveit_commander.RobotCommander()
        
        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        self.scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.  In this case the group is the joints in the Panda
        ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
        ## you should change this value to the name of your robot arm planning group.
        ## This interface can be used to plan and execute motions on the Panda:
        self.group_name = group_name
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        rospy.sleep(0.5)

        self.configs = rospy.get_param("/bark_slidebot_config")
        self.bbox_client = actionlib.SimpleActionClient('bounding_box_detection', BoundingBoxDetectionAction)
        self.bbox_client.wait_for_server()
        self.bridge = CvBridge()
    
    def print_configs(self):
        rospy.loginfo(self.configs)

    def move_to_photo_pose(self):
        photo_pose = self.configs["photo_jpose"]
        rospy.loginfo("Moving to photo pose...")

        joint_goal = JointState()
        joint_goal.name = self.group.get_active_joints()
        joint_values = photo_pose
        joint_goal.position = joint_values

        if not joint_values is None:
            success, plan_trajectory, planning_time, error_code = self.group.plan(joint_goal)
            if success:
                self.group.execute(plan_trajectory, wait = True)

    def bbox_prediction(self, img):
        # TODO: add python packages to Dockerfile
        goal = BoundingBoxDetectionGoal()
        goal.image = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        # Fill in the goal here
        self.bbox_client.send_goal(goal)
        self.bbox_client.wait_for_result(rospy.Duration.from_sec(5.0))
        return self.bbox_client.get_result()

    def deproject_2d_point_to_3d(self):
        # TODO
        pass

    def move_to_pose_with_camera(self, pose):
        # TODO
        pass

if __name__=="__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('bark_slidebot', anonymous=True)
    
    bark_slidebot = BarkSlidebot()
    bark_slidebot.move_to_photo_pose()
    img = cv2.imread(os.path.join(rospackage_root, '..', '4_Color.png'))
    bark_slidebot.bbox_prediction(img)