Header header

uint8 COMMAND_WAIT = 0
uint8 COMMAND_NAVIGATE = 1
uint8 COMMAND_TIP = 2
uint8 COMMAND_ABORT = 3

uint8 command

# The path for the robot to navigate is a list of strings with the names of the via points.
# The naming of the areas may be updated later. The latest definition is available here:
# https://docs.google.com/document/d/1OtfhkGpepaSPo6wy-qbTIneeWb4-85rDyVwZILkzZNk/edit?usp=sharing
# Note that the path is sorted, so the first item is the first location to navigate.
string path
