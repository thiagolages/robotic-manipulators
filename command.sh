#!/bin/bash

# Start robot control script
/home/comau/venv/bin/python $HOME/robotic-manipulators/src/control.py &

# Start CoppeliaSim
/home/comau/start_coppeliasim.sh