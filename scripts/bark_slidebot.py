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
from bark_msgs.msg import BoundingBoxDetectionAction, BoundingBoxDetectionGoal, CameraProjectionsAction, CameraProjectionsGoal
from cv_bridge import CvBridge
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from sensor_msgs.msg import JointState
import copy

import tf
import tf2_ros
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
        self.camera_projections_client = actionlib.SimpleActionClient('camera_projections', CameraProjectionsAction)
        self.bbox_client.wait_for_server()
        self.camera_projections_client.wait_for_server()
        self.bridge = CvBridge()

        self.tf_listener = tf.TransformListener()

        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    def print_configs(self):
        rospy.loginfo(self.configs)

    def move_to_joint_pose(self, pose_name):
        photo_pose = self.configs[pose_name]
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
        goal = BoundingBoxDetectionGoal()
        goal.image = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        # Fill in the goal here
        self.bbox_client.send_goal(goal)
        self.bbox_client.wait_for_result(rospy.Duration.from_sec(10.0))
        return self.bbox_client.get_result()

    def deproject_2d_points_to_3d(self, points):
        goal = CameraProjectionsGoal()
        goal.space_type = goal.IMAGE_2D
        goal.header.frame_id = "base"

        for point in points:
            p = geometry_msgs.msg.Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            goal.points.append(p)
        
        self.camera_projections_client.send_goal(goal)
        self.camera_projections_client.wait_for_result(rospy.Duration.from_sec(5.0))

        return self.camera_projections_client.get_result()


    def move_to_pose_with_frame(self, pose):
        '''
        pose (geometry_msgs/PoseStamped)
        '''
        # We can get the name of the reference frame for this robot:
        print("planning_frame: ", self.group.get_planning_frame())
        transformer = tf.Transformer(True, rospy.Duration(10.0))
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.frame_id = "world"
        transform.child_frame_id = "base_link"
        transform.transform.rotation.w = 1
        transformer.setTransform(transform)

        transform = geometry_msgs.msg.TransformStamped()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "base"
        transform.transform.rotation.z = 1
        transformer.setTransform(transform)

        transform = geometry_msgs.msg.TransformStamped()
        transform.header.frame_id = "base"
        transform.child_frame_id = "target"
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        transformer.setTransform(transform)

        transform = geometry_msgs.msg.TransformStamped()
        transform.header.frame_id = "target"
        eef_link = self.group.get_end_effector_link()
        transform.child_frame_id = eef_link
        trans, rot = self.lookup_transform(eef_link, pose.header.frame_id)
        transform.transform.translation = geometry_msgs.msg.Vector3(*trans)
        transform.transform.rotation = geometry_msgs.msg.Quaternion(*rot)
        print("transform: ", transform)
        transformer.setTransform(transform)

        trans, rot = transformer.lookupTransform("base_link", eef_link, rospy.Time(0))
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position = geometry_msgs.msg.Point(*trans)
        pose_goal.orientation = geometry_msgs.msg.Quaternion(*rot)

        waypoints = []
        waypoints.append(self.group.get_current_pose().pose)
        waypoints.append(copy.deepcopy(pose_goal))

        (plan_trajectory, fraction) = self.group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold

        input("Trajectory computation to target pose finished, observe the result trajectory and press ENTER!")

        if fraction == 1:
            self.group.execute(plan_trajectory, wait = True)

    def lookup_transform(self, parent, child):
        if self.tf_listener.frameExists(parent) and self.tf_listener.frameExists(child):
            t = self.tf_listener.getLatestCommonTime(child, parent)
            (trans, rot) = self.tf_listener.lookupTransform(child, parent, t)
            return (trans, rot)
        else:
            return None

    def enable_joint_constraints(self):
        constraints = moveit_msgs.msg.Constraints()
        joint_constraint = moveit_msgs.msg.JointConstraint()

        constraints.name = "limit_joint_movement"

        for joint_name, joint_pose in zip(self.group.get_active_joints(), self.group.get_current_joint_values()):
            joint_constraint.position = joint_pose

            joint_constraint.tolerance_above = pi/4
            joint_constraint.tolerance_below = pi/4

            joint_constraint.weight = 1

            joint_constraint.joint_name = joint_name
            constraints.joint_constraints.append(joint_constraint)

        self.group.set_path_constraints(constraints)

    def disable_joint_constraints(self):
        self.group.set_path_constraints(None)

if __name__=="__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('bark_slidebot', anonymous=True)
    rospack = rospkg.RosPack()
    rospackage_root = rospack.get_path("bark_slidebot")

    bark_slidebot = BarkSlidebot()

    input("Moving to photo pose, press ENTER to continue!")
    bark_slidebot.move_to_joint_pose("photo_jpose")

    input("Take a photo, press ENTER to continue!")
    img = cv2.imread(os.path.join(rospackage_root, '..', '4_Color.png'))

    input("Predict bounding box, press ENTER to continue!")
    # THE IMAGE HAS TO BE RESIZED TO THE RESOLUTION THAT WAS USED DURING THE CALIBRATION!!!!!!!!!!!!!!! CALIBRATION RES: 1280x720
    resized_img = cv2.resize(img, (1280,720), interpolation= cv2.INTER_LINEAR)

    bbox_detection_result = bark_slidebot.bbox_prediction(resized_img)
    bbox = bbox_detection_result.detections.detections[0].bbox
    print("x: ", bbox.center.x, "y: ", bbox.center.y, "size_x: ", bbox.size_x, "size_y: ", bbox.size_y)

    camera_z = bark_slidebot.lookup_transform("base", "camera")[0][2]
    print("camera_z: ", camera_z)
    # Deprojected point is in the "base" frame
    deprojection_result = bark_slidebot.deproject_2d_points_to_3d([[bbox.center.x, bbox.center.y, camera_z]])
    bbox_centerpoint_3d = deprojection_result.points[0]

    bbox_static_transform = geometry_msgs.msg.TransformStamped()
    bbox_static_transform.header.frame_id = 'base_link'
    bbox_static_transform.child_frame_id = 'predicted_object_pose'
    bbox_static_transform.transform.translation.x = -bbox_centerpoint_3d.x
    bbox_static_transform.transform.translation.y = -bbox_centerpoint_3d.y
    bbox_static_transform.transform.translation.z = bbox_centerpoint_3d.z
    bbox_static_transform.transform.rotation.w = 1
    bark_slidebot.static_tf_broadcaster.sendTransform(bbox_static_transform)

    bbox_centerpoint_3d.z += 0.4
    print("above_bbox_centerpoint_3d: ", bbox_centerpoint_3d)


    rot = bark_slidebot.lookup_transform("base", "camera")[1]
    target_pose = geometry_msgs.msg.PoseStamped()
    target_pose.header.frame_id = "camera"
    target_pose.pose.position = bbox_centerpoint_3d
    target_pose.pose.orientation = geometry_msgs.msg.Quaternion(*rot)

    # bark_slidebot.move_to_joint_pose("pick_jpose")
    bark_slidebot.enable_joint_constraints()
    bark_slidebot.move_to_pose_with_frame(target_pose)
    bark_slidebot.disable_joint_constraints()