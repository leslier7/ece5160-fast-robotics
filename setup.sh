#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Activate the virtual environment using the absolute path
source "$SCRIPT_DIR/FastRobots_ble/bin/activate"

./FastRobots_ble/bin/python3 -m jupyter lab
