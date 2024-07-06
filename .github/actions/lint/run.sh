#!/bin/bash
set -e

./setup.sh

if ["ament_flake8" == "ament_${LINTER}"]
then
    ament_flake8  . --config python_linter.flake8
else
    ament_${LINTER}
fi
