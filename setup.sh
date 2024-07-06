#!/bin/bash
set -e

rosdep update
rosdep install --from-paths . --ignore-src -y
