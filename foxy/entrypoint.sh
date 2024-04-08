#!/bin/bash

set -e

source /opt/ros/foxy/setup.bash

echo "Provided arguments: $@"

exec $@
