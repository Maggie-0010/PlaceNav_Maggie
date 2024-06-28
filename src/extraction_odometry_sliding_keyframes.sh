

#!/bin/bash


# Get the paths to config files
source configs.sh

# Check if the correct number of arguments is provided
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <output_folder> <bag_filename>"
    exit 1
fi

# Assign arguments to variables
OUTPUT_FOLDER="$1"
BAG_FILE="$2"
PYTHON_SCRIPT_PATH="/home/meghana/Documents/PlaceNav/src/My codes/extract_keyframes_and_sliding_window.py" #change this to the actual path of the python file

# Create a new tmux session
session_name="odometry_sampling_$(date +%s)"
tmux new-session -d -s $session_name


# Pane 1: Start roscore
tmux send-keys -t extraction:0 "roscore" C-m

# Split the window horizontally for the second pane
tmux split-window -h -t extraction

# Pane 2: Run the Python script
tmux send-keys -t extraction:0.1 "sleep 5" C-m # Wait for roscore to start
tmux send-keys -t extraction:0.1 "python3 $PYTHON_SCRIPT_PATH \"$BAG_FILE\" \"$OUTPUT_FOLDER\" --trans_threshold 0.3 --rot_threshold 0.2 --window_size 30 --step_size 5" C-m

# Split the second pane vertically for the third pane
tmux split-window -v -t extraction:0.1

# Pane 3: Show progress (for example, using watch or ls)
tmux send-keys -t extraction:0.2 "watch -n 1 ls -l \"$OUTPUT_FOLDER\"" C-m

# Attach to the tmux session
tmux attach-session -t extraction













