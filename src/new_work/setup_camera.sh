#!/bin/bash

# Define the path to the camera check script and configuration file
CAMERA_CHECK_SCRIPT="camera_check.py"
CONFIG_FILE="camera.yaml"

# Function to perform camera check and update camera.yaml
setup_camera() {
    echo "Running camera check..."
    python $CAMERA_CHECK_SCRIPT --config_file $CONFIG_FILE

    # Check if the camera check passed
    WORKING_CAMERA=$(rosparam get /working_camera)

    if [ "$WORKING_CAMERA" == "" ]; then
        echo "No valid camera found. Camera configuration not updated."
        exit 1
    else
        echo "Valid camera found: $WORKING_CAMERA"
    fi

    # Update the camera.yaml file with the working camera device
    echo "Updating camera.yaml with the working camera device..."
    sed -i "s|video_device: \".*\"|video_device: \"$WORKING_CAMERA\"|g" $CONFIG_FILE

    echo "Camera configuration updated successfully."
}

# Run the setup function
setup_camera
