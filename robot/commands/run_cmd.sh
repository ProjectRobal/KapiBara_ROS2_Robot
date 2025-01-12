#!/bin/bash

source /opt/ros/humble/setup.bash
source /app/src/workspace/$INSTALL_LOCATION/setup.bash

exec "$@"