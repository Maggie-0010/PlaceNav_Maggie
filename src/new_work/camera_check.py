import cv2
import os
import yaml
import rospy
import argparse

def load_camera_config(config_file):
    with open(config_file, 'r') as file:
        return yaml.safe_load(file)

def check_camera(video_device, config):
    cap = cv2.VideoCapture(video_device)

    # Apply camera configuration
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config['image_width'])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config['image_height'])
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config['pixel_format']))
    cap.set(cv2.CAP_PROP_FPS, config['framerate'])

    if not cap.isOpened():
        print(f"Error: Could not open video device {video_device}")
        return False

    ret, frame = cap.read()
    cap.release()

    if not ret:
        print(f"Error: Could not read from video device {video_device}")
        return False

    # Check if the frame is green
    if (frame[:, :, 1] > frame[:, :, 0]).all() and (frame[:, :, 1] > frame[:, :, 2]).all():
        print(f"Error: Frame from video device {video_device} is green")
        return False

    # Check if the camera is front-facing
    if is_front_camera(video_device):
        print(f"Error: Video device {video_device} is a front camera")
        return False

    return True

def is_front_camera(video_device):
    # Heuristic to identify a front camera. This can be adapted based on specific characteristics.
    # For demo purposes, we assume video0 and video1 are front cameras
    front_cameras = ["/dev/video0", "/dev/video1"]
    return video_device in front_cameras

def find_working_camera(config):
    video_devices = [f"/dev/video{i}" for i in range(6) if os.path.exists(f"/dev/video{i}")]

    for device in video_devices:
        if check_camera(device, config):
            return device

    return None

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Camera Check Script")
    parser.add_argument("--config_file", type=str, required=True, help="Path to the camera configuration file")
    args = parser.parse_args()

    rospy.init_node('camera_check')
    config = load_camera_config(args.config_file)
    
    working_camera = find_working_camera(config)
    if working_camera:
        rospy.set_param('working_camera', working_camera)
        print(f"Working camera found: {working_camera}")
    else:
        rospy.set_param('working_camera', "")
        print("No working camera found.")
