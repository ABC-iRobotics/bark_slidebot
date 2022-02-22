#! /usr/bin/env python

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Point, TransformStamped, Quaternion


class PublishConfigPosesToTF:
    def __init__(self):
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.transforms = []
        try:
            self.transforms.append(self.transfromstamped_from_array(rospy.get_param('bark_slidebot_config/object_in_world_frame'), 'base_link', 'object'))
            self.transforms.append(self.transfromstamped_from_array(rospy.get_param('bark_slidebot_config/tcp_in_robot_eef_frame'), 'tool0', 'tcp'))
            self.transforms.append(self.transfromstamped_from_array(rospy.get_param('bark_slidebot_config/camera_in_tcp_frame'), 'tcp', 'camera'))
        except:
            rospy.logerr("Could not load transformation parameters 'object_in_world_frame', 'camera_in_tcp_frame', 'tcp_in_robot_eef_frame' ...")
            # TODO default_val


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
