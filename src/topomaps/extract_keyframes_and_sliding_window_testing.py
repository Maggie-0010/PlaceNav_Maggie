'''
import rosbag
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
import os


def count_total_frames(bag_filename):
    bag = rosbag.Bag(bag_filename, 'r')
    frame_count = 0
    for topic, msg, t in bag.read_messages(topics=['/usb_cam/image_raw']):
        frame_count += 1
            
    bag.close()
    return frame_count


#def extract_keyframes_and_sliding_window(bag_filename, output_dir, trans_threshold=0.3, rot_threshold=0.2, window_size=30, step_size=5):
def extract_keyframes_and_sliding_window(bag_filename, output_dir, trans_threshold, rot_threshold, window_size, step_size):
    bridge = CvBridge()
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    
    total_frames = count_total_frames(bag_filename)
    print(f"Total number of frames in the ROS bag: {total_frames}")

    
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
'''

#ABOVE CODE WORKS NOW BUT HAVE TO CHECK WHY THE EXTRACTION OF IMAGES ISN'T INCREASING WHEN THE STEP COUNT DECREASES
'''

import rosbag
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
import os

def count_total_frames(bag_filename):
    bag = rosbag.Bag(bag_filename, 'r')
    frame_count = 0
    for topic, msg, t in bag.read_messages(topics=['/usb_cam/image_raw']):
        frame_count += 1
    bag.close()
    return frame_count

def extract_keyframes_and_sliding_window(bag_filename, output_dir, trans_threshold, rot_threshold, window_size, step_size):
    bridge = CvBridge()
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Count total frames in the bag file
    total_frames = count_total_frames(bag_filename)
    print(f"Total number of frames in the ROS bag: {total_frames}")

    bag = rosbag.Bag(bag_filename, 'r')
    last_pose = None
    frame_count = 0
    keyframe_indices = []

    # Iterate through the messages in the bag to determine keyframes
    for topic, msg, t in bag.read_messages(topics=['/odom']):
        if topic == '/odom':
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            
            # Calculate translational and rotational distances
            if last_pose:
                trans_dist = ((position.x - last_pose[0]) ** 2 + (position.y - last_pose[1]) ** 2) ** 0.5
                rot_dist = abs(orientation.z - last_pose[2])
                
                if trans_dist > trans_threshold or rot_dist > rot_threshold:
                    keyframe_indices.append(frame_count)
                    last_pose = (position.x, position.y, orientation.z)
            else:
                last_pose = (position.x, position.y, orientation.z)
            
            frame_count += 1

    print(f"Number of keyframes determined: {len(keyframe_indices)}")
    print(f"Keyframe indices: {keyframe_indices}")

    # Ensure keyframes are within the sliding window
    keyframe_indices = [idx for idx in keyframe_indices if idx % step_size == 0]

    print(f"Number of keyframes after applying step size: {len(keyframe_indices)}")
    print(f"Keyframe indices after applying step size: {keyframe_indices}")

    # Reset the frame count for image extraction
    frame_count = 0
    extracted_count = 0
    for topic, msg, t in bag.read_messages(topics=['/usb_cam/image_raw']):
        if frame_count in keyframe_indices:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(os.path.join(output_dir, f"frame_{frame_count:06d}.jpg"), cv_image)
            extracted_count += 1
        frame_count += 1
    
    bag.close()
    print(f"Total images extracted: {extracted_count}")

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


    

    #CODE WITH EULER FORMAT FROM QUATERNION form 
'''


import rosbag
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
import os
import math
import numpy as np

def euler_from_quaternion(quat):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    quat is a geometry_msgs/Quaternion
    """
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

def count_total_frames(bag_filename):
    bag = rosbag.Bag(bag_filename, 'r')
    frame_count = 0
    for topic, msg, t in bag.read_messages(topics=['/usb_cam/image_raw']):
        frame_count += 1
    bag.close()
    return frame_count

def extract_keyframes_and_sliding_window(bag_filename, output_dir, trans_threshold, rot_threshold, window_size, step_size):
    bridge = CvBridge()
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Count total frames in the bag file
    total_frames = count_total_frames(bag_filename)
    print(f"Total number of frames in the ROS bag: {total_frames}")

    bag = rosbag.Bag(bag_filename, 'r')
    last_pose = None
    frame_count = 0
    keyframe_indices = []
    last_image = None

    # Iterate through the messages in the bag to determine keyframes
    for topic, msg, t in bag.read_messages(topics=['/odom']):
        if topic == '/odom':
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation

            # Convert quaternion to Euler angles
            _, _, yaw = euler_from_quaternion(orientation)

            # Calculate translational and rotational distances
            if last_pose:
                trans_dist = ((position.x - last_pose[0]) ** 2 + (position.y - last_pose[1]) ** 2) ** 0.5
                rot_dist = abs(yaw - last_pose[2])

                # Consider small angular changes and significant rotations
                if trans_dist > trans_threshold or rot_dist > rot_threshold or abs(rot_dist - math.pi/2) < 0.1:
                    keyframe_indices.append(frame_count)
                    last_pose = (position.x, position.y, yaw)
            else:
                last_pose = (position.x, position.y, yaw)
            
            frame_count += 1

    # Reset the frame count for image extraction
    frame_count = 0
    for topic, msg, t in bag.read_messages(topics=['/usb_cam/image_raw']):
        if frame_count in keyframe_indices:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Check if there is significant change in the image compared to the last image
            if last_image is not None:
                diff = cv2.absdiff(cv_image, last_image)
                non_zero_count = np.count_nonzero(diff)
                if non_zero_count > 1000:  # Arbitrary threshold for significant change
                    cv2.imwrite(os.path.join(output_dir, f"frame_{frame_count:06d}.jpg"), cv_image)
                    last_image = cv_image
            else:
                cv2.imwrite(os.path.join(output_dir, f"frame_{frame_count:06d}.jpg"), cv_image)
                last_image = cv_image
                
        frame_count += 1
    
    bag.close()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Extract keyframes using odometry data.')
    parser.add_argument('bag_filename', help='Input ROS bag file')
    parser.add_argument('output_dir', help='Output directory for extracted images')
    parser.add_argument('--trans_threshold', type=float, default=0.3, help='Translational distance threshold')
    parser.add_argument('--rot_threshold', type=float, default=0.05, help='Rotational distance threshold')  # Lowered to capture small angular changes
    parser.add_argument('--window_size', type=int, default=30, help='Sliding window size')
    parser.add_argument('--step_size', type=int, default=5, help='Sliding window step size')
    args = parser.parse_args()

    extract_keyframes_and_sliding_window(args.bag_filename, args.output_dir, args.trans_threshold, args.rot_threshold, args.window_size, args.step_size)

