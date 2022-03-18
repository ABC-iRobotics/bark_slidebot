#! /usr/bin/env python

import rospy
import moveit_commander
import rospkg
import actionlib
from bark_msgs.msg import BoundingBoxDetectionAction, BoundingBoxDetectionGoal, CameraProjectionsAction, CameraProjectionsGoal, RaspberryPiCameraAction, RaspberryPiCameraGoal, OrientCorrectionAction, OrientCorrectionGoal
from bark_msgs.msg import LaserModulAction, LaserModulGoal
from bark_msgs.msg import SlideDetectionAction, SlideDetectionGoal
from ur_msgs.srv import SetIO, SetIORequest
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
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image

class BarkSlidebot:
    '''
    Class for picking up glass slides
    '''
    def __init__(self, group_name="manipulator"):
        '''
        Constructor of BarkSlidebot class

        arguments:
         - group_name (string): name of the group
        '''
        # Instantiate a RobotCommander object. This object is the outer-level interface to the robot
        self.robot = moveit_commander.RobotCommander()
        
        # Instantiate a PlanningSceneInterface object.  This object is an interface to the world surrounding the robot
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Specify the group name for which to construct this commander instance
        self.group_name = group_name
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        rospy.sleep(0.5)

        self.configs = rospy.get_param("/bark_slidebot_config")

        self.bbox_client = actionlib.SimpleActionClient('bounding_box_detection', BoundingBoxDetectionAction)
        self.camera_projections_client = actionlib.SimpleActionClient('camera_projections', CameraProjectionsAction)
        self.camera_client = actionlib.SimpleActionClient('raspberry_pi_camera', RaspberryPiCameraAction)
        self.orient_client = actionlib.SimpleActionClient('orient_correction', OrientCorrectionAction)
        self.laser_modul_client = actionlib.SimpleActionClient('laser_modul', LaserModulAction)
        self.slide_detection_client = actionlib.SimpleActionClient('slide_detection', SlideDetectionAction)
        self.slide_detection_client.wait_for_server()
        self.laser_modul_client.wait_for_server()
        self.bbox_client.wait_for_server()
        self.camera_projections_client.wait_for_server()
        self.camera_client.wait_for_server()

        self.bridge = CvBridge()

        self.set_io_client = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        try:
            self.set_io_client.wait_for_service(timeout=None)
            self.using_io_client = True
        except rospy.exceptions.ROSException:
            rospy.logerr("Could not reach SetIO service. Make sure that the driver is actually running.")
            self.using_io_client = False

        self.tf_listener = tf.TransformListener()
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        table_pose_stamped = geometry_msgs.msg.PoseStamped()
        table_pose_stamped.header.frame_id = self.robot.get_planning_frame()
        rospy.sleep(0.2)
        table_pose_stamped.pose.position.z = -0.035
        self.scene.add_box("table", table_pose_stamped, (2, 2, 0.05))
      
    
    def print_configs(self):
        '''
        Prints the elements in config file.
        '''
        rospy.loginfo(self.configs)

    def move_to_joint_pose(self, pose_name):
        '''
        Moves the robot to a specified joint pose.

        arguments:
         - pose_name (string): name of the pose where move the robot. The joint variables comes from the config file by name
        '''
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
        '''
        Predicts the bounding box of slide holder box.

        arguments:
            - img (cv::Mat): OpenCV image
        
        return:
            - (vision_msgs/Detection2DArray): Detected objects 
        '''
        goal = BoundingBoxDetectionGoal()
        goal.image = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        # Fill in the goal here
        self.bbox_client.send_goal(goal)
        self.bbox_client.wait_for_result(rospy.Duration.from_sec(10.0))
        return self.bbox_client.get_result()

    def deproject_2d_points_to_3d(self, points):
        '''
        Deproject 2D points to 3D in the robot base frame.

        arguments:
            - points (list[list]): List of x, y, z coordinates of the points.
        
        return:
            - (geometry_msgs/Point[]): List of deprojected points.
        '''
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
        Moves the specified frame to the specified position.

        arguments:
            - pose (geometry_msgs/PoseStamped): The frame is specified in PoseStamped header.frame_id and the position and orientation in pose.
        '''
        # We can get the name of the reference frame for this robot:
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

        input("-----------------Trajectory computation to target pose finished, observe the result trajectory and press ENTER!-----------------")

        if fraction == 1:
            self.group.execute(plan_trajectory, wait = True)

    def lookup_transform(self, parent, child):
        '''
        Gives the translation and orientation of child frame in parent frame.

        arguments:
            - parent, child (string): Lname of the parent and child frames.
        
        return:
            - trans (list): the translation (x, y, z in meter) of child frame in parent frame.
            - rot (quaternion): the orientation quaternion (x, y, z, w) of child frame in parent frame.
        '''
        if self.tf_listener.frameExists(parent) and self.tf_listener.frameExists(child):
            t = self.tf_listener.getLatestCommonTime(child, parent)
            (trans, rot) = self.tf_listener.lookupTransform(child, parent, t)
            return (trans, rot)
        else:
            return None

    def enable_joint_constraints(self):
        '''
        Enables joint contrains for robot.
        '''
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
        '''
        Disables joint contrains for robot.
        '''
        self.group.set_path_constraints(None)

    def get_image(self):
        '''
        Gets image from Raspberry Pi camera.

        return:
            - (cv::Mat): OpenCV image.
        '''
        goal = RaspberryPiCameraGoal()
        # Fill in the goal here
        self.camera_client.send_goal(goal)
        self.camera_client.wait_for_result(rospy.Duration.from_sec(10.0))
        return self.bridge.imgmsg_to_cv2(self.camera_client.get_result().image, "bgr8")

    def orient_correction(self, image):
        '''
        Gives the translation and orientation of child frame in parent frame.

        arguments:
            - (cv::Mat): OpenCV image.
        
        return:
            - (float32): angle of horizon of image and the box in degrees.
        '''
        goal = OrientCorrectionGoal()
        goal.image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        # Fill in the goal here
        self.orient_client.send_goal(goal)
        self.orient_client.wait_for_result(rospy.Duration.from_sec(5.0))
        return self.orient_client.get_result()

    def laser_modul_request(self, on, angle):
        '''
        Turns on or off the laser, and set its angle.

        arguments:
            - on (bool): turn on or off the laser.
            - angle (float32): angle of the laser.
        '''
        goal = LaserModulGoal()
        goal.angle = angle
        goal.on = on
        self.laser_modul_client.send_goal(goal)
        self.laser_modul_client.wait_for_result(rospy.Duration.from_sec(5.0))
        return None

    def slide_detection(self, img_no_laser, img_laser, bbox):
        '''
        Detects the glass slides in the box.

        arguments:
            - img_no_laser (cv::Mat): OpenCV image without laser
            - img_laser (cv::Mat): OpenCV imagewith laser
            - bbox (geometry_msgs/Pose2D ): center, size_x and size_y of the bounding box
        
        return:
            - (geometry_msgs/Pose2D[]): poses of the slides
        '''
        goal = SlideDetectionGoal()
        goal.image_on = self.bridge.cv2_to_imgmsg(img_laser, encoding='bgr8')
        goal.image_off = self.bridge.cv2_to_imgmsg(img_no_laser, encoding='bgr8')
        goal.bbox = bbox
        self.slide_detection_client.send_goal(goal)
        self.slide_detection_client.wait_for_result(rospy.Duration.from_sec(5.0))
        return self.slide_detection_client.get_result()

# IO
    def set_io(self, pin, state):
        '''
        Sets the digital I/O of robot.

        argument:
            - pin (int): number of digital I/O
            - state (bool): value of digital I/O
        '''
        if self.using_io_client:
            return self.set_io_client(1, pin, state)
        else:
            return False

    def move_relative(self, xyz_relative_movement):
        '''
        Moves the robot TCP relative.

        arguments:
            - xyz_relative_movement (list[int]): relative movement of robot TCP
        '''
        waypoints = []
        start_pose = self.group.get_current_pose().pose
        end_pose = copy.deepcopy(start_pose)
        end_pose.position.x += xyz_relative_movement[0]
        end_pose.position.y += xyz_relative_movement[1]
        end_pose.position.z += xyz_relative_movement[2]
        waypoints.append(start_pose)
        waypoints.append(end_pose)

        (plan_trajectory, fraction) = self.group.compute_cartesian_path(
                            waypoints,   # waypoints to follow
                            0.01,        # eef_step
                            0.0)         # jump_threshold

        input("----------Trajectory computation to target pose finished, observe the result trajectory and press ENTER!----------")

        if fraction == 1:
            self.group.execute(plan_trajectory, wait = True)


if __name__=="__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('bark_slidebot', anonymous=True)
    rospack = rospkg.RosPack()
    rospackage_root = rospack.get_path("bark_slidebot")

    bark_slidebot = BarkSlidebot()

    input("-------------Moving to photo pose, press ENTER to continue!-------------")
    bark_slidebot.move_to_joint_pose("photo_jpose")

    # gripper_pose_stamped = geometry_msgs.msg.PoseStamped()
    # gripper_pose_stamped.header.frame_id = "tool0"
    # rospy.sleep(0.2)
    # gripper_pose_stamped.pose.position.z = 0.10
    # bark_slidebot.scene.attach_box("tool0", "gripper", gripper_pose_stamped, (0.2, 0.2, 0.2), touch_links=[])

    # input("------------Take a photo, press ENTER to continue!------------")
    # img = cv2.imread(os.path.join(rospackage_root, '..', '4_Color.png'))
    print("......Get image in photo pose, PLEASE WAIT!......")
    img = bark_slidebot.get_image()
    plt.imshow(img)
    plt.show()

    print("......Bounding box prediction in progress, PLEASE WAIT!......")
    bbox_detection_result = bark_slidebot.bbox_prediction(img)

    if not len(bbox_detection_result.detections.detections) == 0:
        bbox = bbox_detection_result.detections.detections[0].bbox
        print("------------Prediction was SUCCESSFULL!------------")
        # print("x: ", bbox.center.x, "y: ", bbox.center.y, "size_x: ", bbox.size_x, "size_y: ", bbox.size_y)

        # draw boundning box
        fig, ax = plt.subplots()
        ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        rect = patches.Rectangle((bbox.center.x - bbox.size_x/2, bbox.center.y - bbox.size_y/2), bbox.size_x, bbox.size_y, linewidth=2, edgecolor='r', facecolor='none')
        ax.add_patch(rect)
        plt.show()

        # get camera hight
        camera_z = bark_slidebot.lookup_transform("base", "camera")[0][2]
        # print("camera_z: ", camera_z)
        # Deprojected point is in the "base" frame
        # bbox.center.x = bbox.center.x/1920 * 3280
        # bbox.center.y = bbox.center.y/1080 * 2464

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

        # camera above pose hight
        bbox_centerpoint_3d.z += 0.3
        # print("above_bbox_centerpoint_3d: ", bbox_centerpoint_3d)

        # rot = bark_slidebot.lookup_transform("base", "camera")[1]
        # rot = [ -0.0000016, -0.7071052, -0.0000016, 0.7071084 ]
        # rot = [ 0, 0, 0, 1 ]
        rot = [ -0.7071068, 0.7071068, 0, 0 ]
        above_pose = geometry_msgs.msg.PoseStamped()
        above_pose.header.frame_id = "camera"
        above_pose.pose.position = bbox_centerpoint_3d
        above_pose.pose.orientation = geometry_msgs.msg.Quaternion(*rot)

        bark_slidebot.enable_joint_constraints()
        bark_slidebot.move_to_pose_with_frame(above_pose)
        bark_slidebot.disable_joint_constraints()

        print("......Get image above the box, PLEASE WAIT!......")
        img = bark_slidebot.get_image()
        fig, ax = plt.subplots()
        ax.imshow(img)
        circle = plt.Circle((img.shape[1]/2, img.shape[0]/2), 5, color='r')
        ax.add_patch(circle)
        plt.show()

        orient_correct_result = bark_slidebot.orient_correction(img)
        print("==================orient_angle: ",orient_correct_result)
       
        oriented_pose = geometry_msgs.msg.PoseStamped()
        oriented_pose.header.frame_id = 'camera'
        trans, rot = bark_slidebot.lookup_transform("base", "camera")
        # oriented_pose.pose.position = geometry_msgs.msg.Point(*trans)
        oriented_pose.pose.position = above_pose.pose.position

        orient_correct_result.angle += 90 # TODO: vegyük figyelembe a kamera és a megfogó közötti trafót a szög megadásánál
        if abs(orient_correct_result.angle) > 180:
            if orient_correct_result.angle >= 0:
                orient_correct_result.angle = orient_correct_result.angle % 180
            else:
                orient_correct_result.angle = -(abs(orient_correct_result.angle) % 180)
        
        # q = tf.transformations.quaternion_about_axis(pi*((orient_correct_result.angle)/180),  (0, 0, 1))
        q = tf.transformations.quaternion_about_axis(pi*((-orient_correct_result.angle)/180),  (0, 0, 1))
        q = tf.transformations.quaternion_multiply(rot, q)
        oriented_pose.pose.orientation = geometry_msgs.msg.Quaternion(*q)

        input("------------Moving to corrected orientation, press ENTER to continue!------------")
        bark_slidebot.enable_joint_constraints()
        bark_slidebot.move_to_pose_with_frame(oriented_pose)
        bark_slidebot.disable_joint_constraints()

        # orientacio korrekcio mozgas
        # kep keszites mar a helyes orientacioban laser nelkul
        bark_slidebot.laser_modul_request(False, 0)

        print("......Get no laser image, PLEASE WAIT!......")
        img_no_laser = bark_slidebot.get_image()
        I_ff = Image.fromarray(img_no_laser)        
        fig, ax = plt.subplots()
        ax.imshow(img_no_laser)
        circle = plt.Circle((img_no_laser.shape[1]/2, img_no_laser.shape[0]/2), 5, color='r')
        ax.add_patch(circle)
        plt.show()

        # bark_slidebot.laser_modul_request(True, 0)

        # # kep keszites mar a helyes orientacioban laserrel
        # print("......Get laser image, PLEASE WAIT!......")
        # img_laser = bark_slidebot.get_image()
        # # I_on = Image.fromarray(img_laser)
        # # I_on.save('/home/tirczkas/work/cap/on_0.png')
        # plt.imshow(img_laser)
        # plt.show()

        # bark_slidebot.laser_modul_request(False, 0)

        input("------------Slide detection, press ENTER to continue!------------")

        # last laser modul request (laser off)
        # img_no_laser_resized = cv2.resize(img_no_laser, (1920,1080), interpolation= cv2.INTER_LINEAR)
        # bbox_detection_result = bark_slidebot.bbox_prediction(img_no_laser_resized)
        box_detection_result = bark_slidebot.bbox_prediction(img_no_laser)

        if not len(bbox_detection_result.detections.detections) == 0:
            bbox_oriented = bbox_detection_result.detections.detections[0].bbox

            print("......Get image in oriented pose, PLEASE WAIT!......")
            img_oriented = bark_slidebot.get_image()
            # fig, ax = plt.subplots()
            # ax.imshow(cv2.cvtColor(img_oriented, cv2.COLOR_BGR2RGB))
            # rect = patches.Rectangle((bbox_oriented.center.x - bbox_oriented.size_x/2, bbox_oriented.center.y - bbox_oriented.size_y/2), bbox_oriented.size_x, bbox_oriented.size_y, linewidth=2, edgecolor='r', facecolor='none')
            # ax.add_patch(rect)
            # plt.show()
    
            # slide_poses = bark_slidebot.slide_detection(img_no_laser, img_laser, bbox_detection_result.detections.detections[0])
            # slide_poses = slide_poses.poses
            
            # if not len(slide_poses) == 0:
            if True:
                print("------------Slide detection SUCCESSFULL!------------")
                # selected_slide_index = int(len(slide_poses)/2)

                # print("slide_poses[selected_slide_index] :", slide_poses[selected_slide_index])

                camera_z = bark_slidebot.lookup_transform("base", "camera")[0][2]

                # slide_deprojection_result = bark_slidebot.deproject_2d_points_to_3d([[slide_poses[selected_slide_index].x, slide_poses[selected_slide_index].y, camera_z]])
                # slide_middlepoint_3d = slide_deprojection_result.points[0]

                plt.imshow(img_oriented)
                plt.title("Írd le a detektált pont koordinátáit")
                plt.show()
                print("ide írd be az X koordinátát")
                x_slide_center = float(input())
                print("ide írd be az Y koordinátát")
                y_slide_center = float(input())
                plt.imshow(img_oriented)
                plt.scatter(x_slide_center, y_slide_center, color='red')
                plt.title("Erre a pontra fog ráállni a robot")
                plt.show()
                fake_middle_point = geometry_msgs.msg.Point()
                # fake_middle_point.x = x_slide_center
                # fake_middle_point.y = y_slide_center

                z_slide_center = camera_z - 0.03
                slide_deprojection_result = bark_slidebot.deproject_2d_points_to_3d([[x_slide_center, y_slide_center, z_slide_center]])
                slide_middlepoint_3d = slide_deprojection_result.points[0]


                slide_middlepoint_static_transform = geometry_msgs.msg.TransformStamped()
                slide_middlepoint_static_transform.header.frame_id = 'base_link'
                slide_middlepoint_static_transform.child_frame_id = 'slide_middlepoint'
                slide_middlepoint_static_transform.transform.translation.x = -slide_middlepoint_3d.x
                slide_middlepoint_static_transform.transform.translation.y = -slide_middlepoint_3d.y
                slide_middlepoint_static_transform.transform.translation.z = slide_middlepoint_3d.z
                slide_middlepoint_static_transform.transform.rotation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_multiply([1,0,0,0], q))
                bark_slidebot.static_tf_broadcaster.sendTransform(slide_middlepoint_static_transform)


        # if --> move to bbox center
        # rot = [ -0.7071068, 0.7071068, 0, 0 ]
        rot = [ 0, 1, 0, 0 ]
        q = tf.transformations.quaternion_about_axis(pi*((-orient_correct_result.angle)/180),  (0, 0, 1))
        # q = tf.transformations.quaternion_about_axis(pi*((orient_correct_result.angle)/180),  (0, 0, 1))
        q = tf.transformations.quaternion_multiply(rot, q)

        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.frame_id = "tcp"
        target_pose.pose.position = slide_middlepoint_3d # bbox_centerpoint_3d slide_middlepoint_3d
        # target_pose.pose.position.x -= 0.08
        # target_pose.pose.position.y -= 0.1
        target_pose.pose.position.z += 0.1

        target_pose.pose.orientation = geometry_msgs.msg.Quaternion(*q)

        # --> if
        # target_pose = geometry_msgs.msg.PoseStamped()
        # target_pose.header.frame_id = "tcp"
        input("------------Move to pick pose, press ENTER to continue!------------")
            
        # move to the pick pose================================================================
        bark_slidebot.enable_joint_constraints()
        bark_slidebot.move_to_pose_with_frame(target_pose)
        bark_slidebot.disable_joint_constraints()

        # # uveglemez fole allas (10 cm-re az asztal sikjatol)
        # distance = 0.07 # először csak 1 cm-t menjünk le
        # bark_slidebot.set_io(6, 1)
        # rospy.sleep(0.3)
        # bark_slidebot.set_io(7, 0)
        # rospy.sleep(0.3)
        # bark_slidebot.set_io(6, 0)
        # rospy.sleep(0.3)

        # bark_slidebot.move_relative([0,0,-distance])
        # # rospy.sleep(0.5)
        
        # input("------------Press ENTER to close the gripper!------------")
        # bark_slidebot.set_io(7, 1)
        # rospy.sleep(0.5)
        # bark_slidebot.set_io(7, 0)
        # bark_slidebot.move_relative([0,0,distance])
        # --> if

            # else:
            #     print("------------Slide detection FAILED!------------")
        
        # else:
        #     print("------------Oriented box prediction was FAILED!------------")

    else:
        print("------------Photo pose prediction was FAILED!------------")