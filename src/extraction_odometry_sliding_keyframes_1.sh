#!/bin/bash

# Ensure correct number of arguments
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <output_folder> <bag_filename>"
    exit 1
fi

# Assign arguments to variables
OUTPUT_FOLDER="$1"
BAG_FILE="$2"
PYTHON_SCRIPT_PATH="/home/meghana/Documents/PlaceNav/src/new_work/extract_keyframes_and_sliding_window.py" # Change this to the actual path of your Python script

# Create a new tmux session
session_name="odometry_sampling_$(date +%s)"
tmux new-session -d -s $session_name

# Split the window into three panes
tmux selectp -t 0    # Select the first (0) pane
tmux splitw -v -p 50 # Split it into two halves vertically
tmux selectp -t 0    # Go back to the first pane
tmux splitw -h -p 50 # Split it into two halves horizontally

# Run roscore in the first pane
tmux select-pane -t 0
tmux send-keys "roscore" Enter

# Run the Python script in the second pane
tmux select-pane -t 1
tmux send-keys "sleep 5" Enter # Give roscore some time to start
tmux send-keys "python3 $PYTHON_SCRIPT_PATH \"$BAG_FILE\" \"$OUTPUT_FOLDER\" --trans_threshold 0.3 --rot_threshold 0.2 --window_size 30 --step_size 5" Enter

# Monitor the output folder in the third pane
tmux select-pane -t 2
tmux send-keys "watch -n 1 ls -l \"$OUTPUT_FOLDER\"" Enter

# Attach to the tmux session
tmux -2 attach-session -t $session_name
