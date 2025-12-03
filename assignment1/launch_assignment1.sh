#!/bin/bash

#in directory assignemnt1
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

#in directory ros2_ws
WS_ROOT="$(cd "$SCRIPT_DIR"/../../.. && pwd)"
echo "Workspace root: $WS_ROOT"

#build
cd "$WS_ROOT" || exit 1
echo "Running colcon build for node1_ui and node2_distance..."
colcon build --packages-select node1_ui node2_distance || {
  echo "Build failed, aborting."
  exit 1
}

source "$WS_ROOT/install/setup.bash"

# turtlesim, turtle_spawn, node_distance
echo "Launching turtlesim, turtle_spawn and node_distance..."
ros2 launch node2_distance assignment_launch.py &

sleep 2

#terminal choise (UI)
if command -v gnome-terminal >/dev/null 2>&1; then
  TERM_CMD="gnome-terminal --"
elif command -v xterm >/dev/null 2>&1; then
  TERM_CMD="xterm -e"
else
  echo "new terminal:"
  echo "  source \"$WS_ROOT/install/setup.bash\""
  echo "  ros2 run node1_ui node_ui"
  exit 0
fi

echo "Opening UI node in a new terminal"
$TERM_CMD bash -c "source \"$WS_ROOT/install/setup.bash\"; ros2 run node1_ui node_ui; exec bash"
