#!/bin/bash

status=0 && for f in $HOME/catkin_ws/devel/lib/*/*-test; do $f || exit 1; done
