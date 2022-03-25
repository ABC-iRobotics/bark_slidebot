#! /usr/bin/env python

##  @package bark_slidebot
# The ROS node for publishing the camera pose to tf
#
# Publishes camera pose to tf so we can get transformation easily

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Point, TransformStamped, Quaternion


## PublishConfigPosesToTF class
#
# Publishes the frames defined in the config to tf
class PublishConfigPosesToTF:
    '''
    Class for publishing config frames to tf
    '''
    def __init__(self):
        ## @var broadcaster
        #  tf2_ros.TransformBroadcaster() object for publishing transforms to tf
        self.broadcaster = tf2_ros.TransformBroadcaster()

        ## @var transforms
        #  List of transformations
        self.transforms = []
        try:
            self.transforms.append(self.transfromstamped_from_array(rospy.get_param('bark_slidebot_config/object_in_world_frame'), 'base_link', 'object'))
            self.transforms.append(self.transfromstamped_from_array(rospy.get_param('bark_slidebot_config/tcp_in_robot_eef_frame'), 'tool0', 'tcp'))
            self.transforms.append(self.transfromstamped_from_array(rospy.get_param('bark_slidebot_config/camera_in_tcp_frame'), 'tcp', 'camera'))
        except:
            rospy.logerr("Could not load transformation parameters 'object_in_world_frame', 'camera_in_tcp_frame', 'tcp_in_robot_eef_frame' ...")
            # TODO default_val

    ## Create geometry_msgs/TransformStamped from numpy array or Python list
    # Builds a geometry_msgs/TransformStamped object from a numpy array or Python list and other arguments
    # @param array list or numpy array type pose [x,y,z,rx,ry,rz], (rx,ry,rz are given in Euler angles)
    # @param parent_frame string type name for parent frame
    # @param frame_id string type name for frame_id
    def transfromstamped_from_array(self, array, parent_frame, frame_id):
        if array:
            transform = TransformStamped()
            transform.header.frame_id = parent_frame
            transform.header.stamp = rospy.Time.now()
            transform.child_frame_id = frame_id
            transform.transform.translation.x = array[0]
            transform.transform.translation.y = array[1]
            transform.transform.translation.z = array[2]
            quaternion = tf.transformations.quaternion_from_euler(array[3],array[4],array[5]).tolist()
            transform.transform.rotation = Quaternion(*quaternion)
            return transform
        else:
            return None

    ## Publish frames
    # Publish the transformations stored in transforms to tf
    def publish_frames(self):
        for transform in self.transforms:
            if transform is not None:
                transform.header.stamp = rospy.Time.now()
                self.broadcaster.sendTransform(transform)

if __name__ == '__main__':
    rospy.init_node('publish_config_poses_to_tf')

    publisher = PublishConfigPosesToTF()

    while not rospy.is_shutdown():
        publisher.publish_frames()
