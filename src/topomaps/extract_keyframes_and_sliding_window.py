'''
import rosbag
import cv2
from cv_bridge import CvBridge
import argparse
import os
import numpy as np

def extract_keyframes_and_sliding_window(bag_filename, output_dir, image_topic, odometry_topic, trans_threshold, rot_threshold, window_size, step_size):
    bag = rosbag.Bag(bag_filename, 'r')
    bridge = CvBridge()
    os.makedirs(output_dir, exist_ok=True)
    
    last_pose = None
    count = 0
    images = []
    
    for topic, msg, t in bag.read_messages(topics=[image_topic, odometry_topic]):
        if topic == odometry_topic:
            current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            if last_pose is not None:
                trans_change = np.linalg.norm(np.array(current_pose[:3]) - np.array(last_pose[:3]))
                rot_change = np.linalg.norm(np.array(current_pose[3:]) - np.array(last_pose[3:]))
                
                if trans_change > trans_threshold or rot_change > rot_threshold:
                    last_pose = current_pose
                else:
                    continue
            else:
                last_pose = current_pose

        if topic == image_topic and last_pose:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            images.append(cv_image)
            
            if len(images) == window_size:
                # Extract frames from the sliding window
                for i in range(0, window_size, step_size):
                    image_filename = os.path.join(output_dir, f"frame_{count:06d}.png")
                    cv2.imwrite(image_filename, images[i])
                    count += 1
                
                # Clear images to start a new window
                images = images[step_size:]

    bag.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Extract keyframes and sliding window frames from a ROS bag based on odometry changes.")
    parser.add_argument("bag_filename", help="Input ROS bag file")
    parser.add_argument("output_dir", help="Directory to save extracted images")
    parser.add_argument("--image_topic", default="/camera/image_raw", help="Image topic in the ROS bag")
    parser.add_argument("--odometry_topic", default="/odom", help="Odometry topic in the ROS bag")
    parser.add_argument("--trans_threshold", type=float, default=0.5, help="Translation change threshold")
    parser.add_argument("--rot_threshold", type=float, default=0.1, help="Rotation change threshold")
    parser.add_argument("--window_size", type=int, default=30, help="Sliding window size")
    parser.add_argument("--step_size", type=int, default=5, help="Step size within the window")

    args = parser.parse_args()
    extract_keyframes_and_sliding_window(args.bag_filename, args.output_dir, args.image_topic, args.odometry_topic,
                                         args.trans_threshold, args.rot_threshold, args.window_size, args.step_size)
                                         
                                         
'''

import rosbag
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
import os

#def extract_keyframes_and_sliding_window(bag_filename, output_dir, trans_threshold=0.3, rot_threshold=0.2, window_size=30, step_size=5):
def extract_keyframes_and_sliding_window(bag_filename, output_dir, trans_threshold, rot_threshold, window_size, step_size):
    bridge = CvBridge()
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    bag = rosbag.Bag(bag_filename, 'r')
    last_pose = None
    frame_count = 0
    keyframe_indices = []

    #iiterating through the messages in the bag
    for topic, msg, t in bag.read_messages(topics=['/odom']):
        if topic == '/odom':
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            
            #calculating translational and rotational distances
            if last_pose:
                trans_dist = ((position.x - last_pose[0]) ** 2 + (position.y - last_pose[1]) ** 2) ** 0.5
                rot_dist = abs(orientation.z - last_pose[2])
                
                if trans_dist > trans_threshold or rot_dist > rot_threshold:
                    keyframe_indices.append(frame_count)
                    last_pose = (position.x, position.y, orientation.z)
            else:
                last_pose = (position.x, position.y, orientation.z)
            
            frame_count += 1

    #reset the frame count for image extraction
    frame_count = 0
    for topic, msg, t in bag.read_messages(topics=['/usb_cam/image_raw']):
        if frame_count in keyframe_indices:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(os.path.join(output_dir, f"frame_{frame_count:06d}.jpg"), cv_image)
        frame_count += 1
    
    bag.close()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Extract keyframes using odometry data.')
    parser.add_argument('bag_filename', help='Input ROS bag file')
    parser.add_argument('output_dir', help='Output directory for extracted images')
    parser.add_argument('--trans_threshold', type=float, default=0.3, help='Translational distance threshold')
    parser.add_argument('--rot_threshold', type=float, default=0.2, help='Rotational distance threshold')
    parser.add_argument('--window_size', type=int, default=30, help='Sliding window size')
    parser.add_argument('--step_size', type=int, default=5, help='Sliding window step size')
    args = parser.parse_args()

    extract_keyframes_and_sliding_window(args.bag_filename, args.output_dir, args.trans_threshold, args.rot_threshold, args.window_size, args.step_size)
