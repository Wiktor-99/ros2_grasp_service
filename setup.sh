#!/bin/bash
set -e

rosdep update
rosdep install --from-paths src --ignore-src -y
