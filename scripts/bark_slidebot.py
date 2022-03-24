#! /usr/bin/env python

##  @package bark_slidebot
#   The main program of Slidebot project.

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

##
#   Class for picking up glass slides.
class BarkSlidebot:
    '''
    Class for picking up glass slides.
    '''

    ##  Constructor of BarkSlidebot class.
    #   @param group_name Name of the group. Representation of a set of joints and links. It comes from .srdf file of the robot.
    def __init__(self, group_name="manipulator"):
        '''
        Constructor of BarkSlidebot class

        Arguments:
         - group_name (string): name of the group. Representation of a set of joints and links. It comes from .srdf file of the robot.
        '''
        ## Instantiate a RobotCommander object. This object is the outer-level interface to the robot
        self.robot = moveit_commander.RobotCommander()
        
        ## Instantiate a PlanningSceneInterface object.  This object is an interface to the world surrounding the robot
        self.scene = moveit_commander.PlanningSceneInterface()
        
        ## Specify the group name for which to construct this commander instance
        self.group_name = group_name
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        rospy.sleep(0.5)

        self.configs = rospy.get_param("/bark_slidebot_config")

        ## Instantiate the clients
        self.bbox_client = actionlib.SimpleActionClient('bounding_box_detection', BoundingBoxDetectionAction)
        self.camera_projections_client = actionlib.SimpleActionClient('camera_projections', CameraProjectionsAction)
        self.camera_client = actionlib.SimpleActionClient('raspberry_pi_camera', RaspberryPiCameraAction)
        self.orient_client = actionlib.SimpleActionClient('orient_correction', OrientCorrectionAction)
        self.laser_modul_client = actionlib.SimpleActionClient('laser_modul', LaserModulAction)
        self.slide_detection_client = actionlib.SimpleActionClient('slide_detection', SlideDetectionAction)

        ## Wait for response of the server
        self.slide_detection_client.wait_for_server()
        self.laser_modul_client.wait_for_server()
        self.bbox_client.wait_for_server()
        self.camera_projections_client.wait_for_server()
        self.camera_client.wait_for_server()

        self.bridge = CvBridge()

        ## Instantiate IO client 
        self.set_io_client = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        try:
            self.set_io_client.wait_for_service(timeout=None)
            self.using_io_client = True
        except rospy.exceptions.ROSException:
            rospy.logerr("Could not reach SetIO service. Make sure that the driver is actually running.")
            self.using_io_client = False

        self.tf_listener = tf.TransformListener()
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        ## Define table to robot
        table_pose_stamped = geometry_msgs.msg.PoseStamped()
        table_pose_stamped.header.frame_id = self.robot.get_planning_frame()
        rospy.sleep(0.2)
        table_pose_stamped.pose.position.z = -0.035
        ## Add table to robot scene
        self.scene.add_box("table", table_pose_stamped, (2, 2, 0.05))
      
    ##  Prints the elements from config file.
    #   @param self The object pointer.
    def print_configs(self):
        '''
        Prints the elements from config file.
        '''
        rospy.loginfo(self.configs)

    ##  Moves the robot to a specified joint pose.
    #   @param pose_name Name of the pose where move the robot. The joint variables come from the config file by name.
    def move_to_joint_pose(self, pose_name):
        '''
        Moves the robot to a specified joint pose.

        Arguments:
         - pose_name (string): name of the pose where move the robot. The joint variables come from the config file by name.
        '''
        photo_pose = self.configs[pose_name]

        joint_goal = JointState()
        joint_goal.name = self.group.get_active_joints()
        joint_values = photo_pose
        joint_goal.position = joint_values

        if not joint_values is None:
            success, plan_trajectory, planning_time, error_code = self.group.plan(joint_goal)
            if success:
                self.group.execute(plan_trajectory, wait = True)

    ##  Predicts the bounding box of slide holder box.
    #   @param img The image on which we want to predict.
    #   @return Detected objects in an array.
    def bbox_prediction(self, img):
        '''
        Predicts the bounding box of slide holder box.

        Arguments:
            - img (cv::Mat): OpenCV image. We want to predict on this image.
        
        Returns:
            - (vision_msgs/Detection2DArray): detected objects in an array
        '''
        ## Instantiate the goal
        goal = BoundingBoxDetectionGoal()
        ## Convert OpenCV image to ROS image
        goal.image = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        ## Send the goal and wait for the result
        self.bbox_client.send_goal(goal)
        self.bbox_client.wait_for_result(rospy.Duration.from_sec(10.0))

        return self.bbox_client.get_result()

    ##  Deprojects 2D points to 3D in the robot base frame.
    #   @param points List of x, y, z coordinates of the points. The x, y points are given in pixel unit and z is given in meter.
    #   @return List of deprojected points. The points are given in meter.
    def deproject_2d_points_to_3d(self, points):
        '''
        Deprojects 2D points to 3D in the robot base frame.

        Arguments:
            - points (list[list]): list of x, y, z coordinates of the points. The x, y points are given in pixel unit and z is given in meter.
        
        Returns:
            - (geometry_msgs/Point[]): list of deprojected points. The points are given in meter.
        '''
        ## Instantiate the goal
        goal = CameraProjectionsGoal()
        goal.space_type = goal.IMAGE_2D
        goal.header.frame_id = "base"

        for point in points:
            p = geometry_msgs.msg.Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            goal.points.append(p)
        
        ## Send the goal and wait for the result
        self.camera_projections_client.send_goal(goal)
        self.camera_projections_client.wait_for_result(rospy.Duration.from_sec(5.0))

        return self.camera_projections_client.get_result()


    ##  Moves the specified frame to the specified position.
    #   @param pose The frame is specified in header.frame_id of PoseStamped and the position and orientation in pose of PoseStamped.
    def move_to_pose_with_frame(self, pose):
        '''
        Moves the specified frame to the specified position.

        Arguments:
            - pose (geometry_msgs/PoseStamped): The frame is specified in header.frame_id of PoseStamped and the position and orientation in pose of PoseStamped.
        '''
        ## We can get the name of the reference frame for this robot:
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

    ##  Gives the translation and orientation of child frame in parent frame.
    #   @param parent Name of the parent frame.
    #   @param child Name of the child frame.
    #   @return trans The translation (x, y, z in meter) of child frame in parent frame.
    #   @return rot The orientation of child frame in parent frame represented in quaternion (x, y, z, w).
    #   @return Retruns 'None' if the parent or child frame is not exist.
    def lookup_transform(self, parent, child):
        '''
        Gives the translation and orientation of child frame in parent frame.

        Arguments:
            - parent, child (string): name of the parent and child frames
        
        Returns:
            - trans (list): the translation (x, y, z in meter) of child frame in parent frame
            - rot (quaternion): the orientation of child frame in parent frame represented in quaternion (x, y, z, w)
            - Retruns 'None' if the parent or child frame is not exist.
        '''
        if self.tf_listener.frameExists(parent) and self.tf_listener.frameExists(child):
            t = self.tf_listener.getLatestCommonTime(child, parent)
            (trans, rot) = self.tf_listener.lookupTransform(child, parent, t)
            return (trans, rot)
        else:
            return None

    ##  Enables joint contrains for robot.
    #   @param self The object pointer.
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

    ##  Disables joint contrains for robot.
    #   @param self The object pointer.
    def disable_joint_constraints(self):
        '''
        Disables joint contrains for robot.
        '''
        self.group.set_path_constraints(None)

    ##  Gets image from Raspberry Pi camera.
    #   @param self The object pointer.
    #   @return The captured image.
    def get_image(self):
        '''
        Gets image from Raspberry Pi camera.

        Returns:
            - (cv::Mat): OpenCV image. The captured image.
        '''
        ## Instantiate the goal
        goal = RaspberryPiCameraGoal()
        ## Send the goal and wait for the result
        self.camera_client.send_goal(goal)
        self.camera_client.wait_for_result(rospy.Duration.from_sec(10.0))

        return self.bridge.imgmsg_to_cv2(self.camera_client.get_result().image, "bgr8")

    ##  Gives the translation and orientation of child frame in parent frame.
    #   @param image The image on which we want to calculate the orientation.
    #   @return The angle between the bottom of the image and the box in degrees.
    def orient_correction(self, image):
        '''
        Gives the translation and orientation of child frame in parent frame.

        Arguments:
            - image (cv::Mat): OpenCV image. The image on which we want to calculate the orientation.
        
        Returns:
            - (float32): angle between the bottom of the image and the box in degrees.
        '''
        ## Instantiate the goal
        goal = OrientCorrectionGoal()
        ## Convert OpenCV image to ROS image
        goal.image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        ## Send the goal and wait for the result
        self.orient_client.send_goal(goal)
        self.orient_client.wait_for_result(rospy.Duration.from_sec(5.0))

        return self.orient_client.get_result()

    ##  Turns on or off the laser and sets its angle.
    #   @param on Turn on or off the laser.
    #   @param angle The angle of the laser.
    def laser_modul_request(self, on, angle):
        '''
        Turns on or off the laser and sets its angle.

        Arguments:
            - on (bool): turns on or off the laser.
            - angle (float32): the angle of the laser.
        '''
        ## Instantiate the goal
        goal = LaserModulGoal()
        goal.angle = angle
        goal.on = on
        ## Send the goal and wait for the result
        self.laser_modul_client.send_goal(goal)
        self.laser_modul_client.wait_for_result(rospy.Duration.from_sec(5.0))

        return None

    ##  Detects the glass slides in the box.
    #   @param img_no_laser Image without laser.
    #   @param img_laser Image with laser.
    #   @param bbox The bounding box parameters.
    #   @return The poses of the slides.
    def slide_detection(self, img_no_laser, img_laser, bbox):
        '''
        Detects the glass slides in the box.

        Arguments:
            - img_no_laser (cv::Mat): OpenCV image without laser
            - img_laser (cv::Mat): OpenCV image with laser
            - bbox (geometry_msgs/Pose2D ): center, size_x and size_y of the bounding box
        
        Returns:
            - (geometry_msgs/Pose2D[]): poses of the slides
        '''
        ## Instantiate the goal
        goal = SlideDetectionGoal()
        goal.bbox = bbox
        ## Convert OpenCV images to ROS images
        goal.image_on = self.bridge.cv2_to_imgmsg(img_laser, encoding='bgr8')
        goal.image_off = self.bridge.cv2_to_imgmsg(img_no_laser, encoding='bgr8')
        ## Send the goal and wait for the result
        self.slide_detection_client.send_goal(goal)
        self.slide_detection_client.wait_for_result(rospy.Duration.from_sec(5.0))

        return self.slide_detection_client.get_result()

    ##  Sets the digital I/O of robot.
    #   @param pin The number of digital I/O.
    #   @param state The value of digital I/O.
    #   @return Returns True if the setting was successfull and returns False if it failed.
    def set_io(self, pin, state):
        '''
        Sets the digital I/O of robot.

        Arguments:
            - pin (int): number of digital I/O
            - state (bool): value of digital I/O
        
        Returns:
            - (bool): Returns True if the setting was successfull and returns False if it failed.
        '''
        if self.using_io_client:
            return self.set_io_client(1, pin, state)
        else:
            return False

    ##  Moves the robot TCP relative.
    #   @param xyz_relative_movement Relative movement of robot TCP.
    def move_relative(self, xyz_relative_movement):
        '''
        Moves the robot TCP relative.

        Arguments:
            - xyz_relative_movement (list): relative movement of robot TCP
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

    ## Instantiate BarkSlidebot class
    bark_slidebot = BarkSlidebot()

    input("-------------Moving to photo pose, press ENTER to continue!-------------")
    bark_slidebot.move_to_joint_pose("photo_jpose")

    # ## Define a gripper covering box
    # gripper_pose_stamped = geometry_msgs.msg.PoseStamped()
    # gripper_pose_stamped.header.frame_id = "tool0"
    # rospy.sleep(0.2)
    # gripper_pose_stamped.pose.position.z = 0.10
    # ## Attach the gripper covering box to robot TCP
    # bark_slidebot.scene.attach_box("tool0", "gripper", gripper_pose_stamped, (0.2, 0.2, 0.2), touch_links=[])

    # ## Read photo from file
    # img = cv2.imread(os.path.join(rospackage_root, '..', '4_Color.png'))

    ## Get image in photo pose
    print("......Get image in photo pose, PLEASE WAIT!......")
    img = bark_slidebot.get_image()
    plt.imshow(img)
    plt.show()

    ## Predict bounding box on image which was taken in the photo pose
    print("......Bounding box prediction in progress, PLEASE WAIT!......")
    bbox_detection_result = bark_slidebot.bbox_prediction(img)

    ## If there is at least one detected box
    if not len(bbox_detection_result.detections.detections) == 0:
        bbox = bbox_detection_result.detections.detections[0].bbox
        print("------------Prediction was SUCCESSFULL!------------")

        ## Draw bounding box
        fig, ax = plt.subplots()
        ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        rect = patches.Rectangle((bbox.center.x - bbox.size_x/2, bbox.center.y - bbox.size_y/2), bbox.size_x, bbox.size_y, linewidth=2, edgecolor='r', facecolor='none')
        ax.add_patch(rect)
        plt.show()

        ## Get camera hight
        camera_z = bark_slidebot.lookup_transform("base", "camera")[0][2]

        ## Deporoject the center of the detected bounding box to 3D space. We get the result in robot base frame.
        deprojection_result = bark_slidebot.deproject_2d_points_to_3d([[bbox.center.x, bbox.center.y, camera_z]])
        bbox_centerpoint_3d = deprojection_result.points[0]

        ## Place a coordinate system to the scene to visualize the location of holder box
        bbox_static_transform = geometry_msgs.msg.TransformStamped()
        bbox_static_transform.header.frame_id = 'base_link'
        bbox_static_transform.child_frame_id = 'predicted_object_pose'
        bbox_static_transform.transform.translation.x = -bbox_centerpoint_3d.x
        bbox_static_transform.transform.translation.y = -bbox_centerpoint_3d.y
        bbox_static_transform.transform.translation.z = bbox_centerpoint_3d.z
        bbox_static_transform.transform.rotation.w = 1
        bark_slidebot.static_tf_broadcaster.sendTransform(bbox_static_transform)

        ## Define the camera above pose hight
        bbox_centerpoint_3d.z += 0.3

        ## Calculate the above pose position and orientation
        # rot = bark_slidebot.lookup_transform("base", "camera")[1]
        rot = [ -0.7071068, 0.7071068, 0, 0 ]
        above_pose = geometry_msgs.msg.PoseStamped()
        above_pose.header.frame_id = "camera"
        above_pose.pose.position = bbox_centerpoint_3d
        above_pose.pose.orientation = geometry_msgs.msg.Quaternion(*rot)

        ## Enable joint constraints, move the camera above the box, then disable joint constraits
        bark_slidebot.enable_joint_constraints()
        bark_slidebot.move_to_pose_with_frame(above_pose)
        bark_slidebot.disable_joint_constraints()

        ## Get image above the box
        print("......Get image above the box, PLEASE WAIT!......")
        img = bark_slidebot.get_image()
        fig, ax = plt.subplots()
        ax.imshow(img)
        ## Display the center of the image
        circle = plt.Circle((img.shape[1]/2, img.shape[0]/2), 5, color='r')
        ax.add_patch(circle)
        plt.show()

        ## Get the angle which we want correct the orientation
        orient_correct_result = bark_slidebot.orient_correction(img)
        print("================== orient_angle: ",orient_correct_result)
       
        ## Calculate the oriented pose position and orientation
        oriented_pose = geometry_msgs.msg.PoseStamped()
        oriented_pose.header.frame_id = 'camera'
        trans, rot = bark_slidebot.lookup_transform("base", "camera")
        # oriented_pose.pose.position = geometry_msgs.msg.Point(*trans)
        oriented_pose.pose.position = above_pose.pose.position

        orient_correct_result.angle += 90 # TODO: consider the transform between the camera and the gripper when specifying the angle

        ## Check the rotating angle. If it is greather than 180 degrees, we get the (angle - 180) degrees.
        if abs(orient_correct_result.angle) > 180:
            if orient_correct_result.angle >= 0:
                orient_correct_result.angle = orient_correct_result.angle % 180
            else:
                orient_correct_result.angle = -(abs(orient_correct_result.angle) % 180)
        
        q = tf.transformations.quaternion_about_axis(pi*((-orient_correct_result.angle)/180),  (0, 0, 1))
        q = tf.transformations.quaternion_multiply(rot, q)
        oriented_pose.pose.orientation = geometry_msgs.msg.Quaternion(*q)


        ## Enable joint constraints, rotate the robot around the camera axis to oriented pose, then disable joint constraits
        input("------------Moving to corrected orientation, press ENTER to continue!------------")
        bark_slidebot.enable_joint_constraints()
        bark_slidebot.move_to_pose_with_frame(oriented_pose)
        bark_slidebot.disable_joint_constraints()

        ## Turn off the laser
        bark_slidebot.laser_modul_request(False, 0)

        ## Get image without laser line
        print("......Get no laser image, PLEASE WAIT!......")
        img_no_laser = bark_slidebot.get_image()
        I_ff = Image.fromarray(img_no_laser)        
        fig, ax = plt.subplots()
        ax.imshow(img_no_laser)
        ## Display the center of the image
        circle = plt.Circle((img_no_laser.shape[1]/2, img_no_laser.shape[0]/2), 5, color='r')
        ax.add_patch(circle)
        plt.show()

        ## Turn on the laser
        bark_slidebot.laser_modul_request(True, 0)

        ## Get image with laser line
        print("......Get laser image, PLEASE WAIT!......")
        img_laser = bark_slidebot.get_image()
        plt.imshow(img_laser)
        plt.show()

        # I_on = Image.fromarray(img_laser)
        # I_on.save('/home/tirczkas/work/cap/on_0.png')

        ## Turn off the laser
        bark_slidebot.laser_modul_request(False, 0)

        input("------------Start slide detection, press ENTER to continue!------------")
        ## Predict bounding box on image which was taken above the box and in the correct orientation
        box_detection_result = bark_slidebot.bbox_prediction(img_no_laser)

        ## If there is a detected box
        if not len(bbox_detection_result.detections.detections) == 0:
            ## Get the parameters of the bounding box
            bbox_oriented = bbox_detection_result.detections.detections[0].bbox

            ## Draw predicted bounding box
            fig, ax = plt.subplots()
            ax.imshow(cv2.cvtColor(img_no_laser, cv2.COLOR_BGR2RGB))
            rect = patches.Rectangle((bbox_oriented.center.x - bbox_oriented.size_x/2, bbox_oriented.center.y - bbox_oriented.size_y/2), bbox_oriented.size_x, bbox_oriented.size_y, linewidth=2, edgecolor='r', facecolor='none')
            ax.add_patch(rect)
            plt.show()

            ## Detect slides in holder box
            slide_poses = bark_slidebot.slide_detection(img_no_laser, img_laser, bbox_detection_result.detections.detections[0])
            slide_poses = slide_poses.poses
            
            ## If there is at least one slide
            if not len(slide_poses) == 0:
                print("------------Slide detection SUCCESSFULL!------------")
                ## Select the slide from the middle of the list
                selected_slide_index = int(len(slide_poses)/2)

                ## Get camera hight
                camera_z = bark_slidebot.lookup_transform("base", "camera")[0][2]
                
                ## Deporoject the middle point of the detected slide to 3D space. We get the result in robot base frame.
                slide_deprojection_result = bark_slidebot.deproject_2d_points_to_3d([[slide_poses[selected_slide_index].x, slide_poses[selected_slide_index].y, camera_z]])
                ## Get the middle point of the slide
                slide_middlepoint_3d = slide_deprojection_result.points[0]

                # plt.imshow(img_no_laser)
                # plt.title("Írd le a detektált pont koordinátáit")
                # plt.show()
                # print("ide írd be az X koordinátát")
                # x_slide_center = float(input())
                # print("ide írd be az Y koordinátát")
                # y_slide_center = float(input())
                # plt.imshow(img_no_laser)
                # plt.scatter(x_slide_center, y_slide_center, color='red')
                # plt.title("Erre a pontra fog ráállni a robot")
                # plt.show()
                # fake_middle_point = geometry_msgs.msg.Point()
                # # fake_middle_point.x = x_slide_center
                # # fake_middle_point.y = y_slide_center

                # z_slide_center = camera_z - 0.03
                # slide_deprojection_result = bark_slidebot.deproject_2d_points_to_3d([[x_slide_center, y_slide_center, z_slide_center]])
                # slide_middlepoint_3d = slide_deprojection_result.points[0]

                ## Place a coordinate system to the scene to visualize the location of slide middle point
                slide_middlepoint_static_transform = geometry_msgs.msg.TransformStamped()
                slide_middlepoint_static_transform.header.frame_id = 'base_link'
                slide_middlepoint_static_transform.child_frame_id = 'slide_middlepoint'
                slide_middlepoint_static_transform.transform.translation.x = -slide_middlepoint_3d.x
                slide_middlepoint_static_transform.transform.translation.y = -slide_middlepoint_3d.y
                slide_middlepoint_static_transform.transform.translation.z = slide_middlepoint_3d.z
                slide_middlepoint_static_transform.transform.rotation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_multiply([1,0,0,0], q))
                bark_slidebot.static_tf_broadcaster.sendTransform(slide_middlepoint_static_transform)

                ## Define the position of the tartget pose
                target_pose = geometry_msgs.msg.PoseStamped()
                target_pose.header.frame_id = "tcp"
                target_pose.pose.position = slide_middlepoint_3d # bbox_centerpoint_3d slide_middlepoint_3d
                ## Go 10 cm above the slide with gripper 
                target_pose.pose.position.z += 0.1

                ## Define the orientation of the tartget pose
                rot = [ 0, 1, 0, 0 ]
                q = tf.transformations.quaternion_about_axis(pi*((-orient_correct_result.angle)/180),  (0, 0, 1))
                q = tf.transformations.quaternion_multiply(rot, q)
                target_pose.pose.orientation = geometry_msgs.msg.Quaternion(*q)

                ## Enable joint constraints, move to target pose with gripper, then disable joint constraits
                input("------------Move to pick pose, press ENTER to continue!------------")
                bark_slidebot.enable_joint_constraints()
                bark_slidebot.move_to_pose_with_frame(target_pose)
                bark_slidebot.disable_joint_constraints()

                distance = 0.03 # distance by which we move down in meter

                ## Open the gripper
                bark_slidebot.set_io(6, 1)
                rospy.sleep(0.3)
                bark_slidebot.set_io(7, 0)
                rospy.sleep(0.3)
                bark_slidebot.set_io(6, 0)
                rospy.sleep(0.3)

                ## Move down relatively with the set distance
                bark_slidebot.move_relative([0,0,-distance])
                rospy.sleep(0.5)
                
                ## Close the gripper
                input("------------Press ENTER to close the gripper!------------")
                bark_slidebot.set_io(7, 1)
                rospy.sleep(0.5)
                bark_slidebot.set_io(7, 0)

                ## Move up relatively with the set distance
                bark_slidebot.move_relative([0,0,distance])

            else:
                ## If there is not detected slide
                print("------------Slide detection FAILED!------------")
        
        else:
            ## If there is not detected box in oriented pose
            print("------------Oriented box prediction was FAILED!------------")

    else:
        ## If there is not detected box in photot pose
        print("------------Photo pose prediction was FAILED!------------")