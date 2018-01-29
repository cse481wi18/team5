#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from annotator import Annotator
import pickle
import copy
import os
import rospy

help_message = """Welcome to the map annotator!"""
commands_msg = """Commands:
  list: List saved annotator.
  save <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists.
  delete <name>: Delete the pose given by <name>.
  goto <name>: Sends the robot to the pose given by <name>.
  help: Show this list of commands
"""

#FILE_NAME="pickled"

def get_command():
    return raw_input('> ').split(" ", 1)

def main():
    print help_message
    print commands_msg
    rospy.init_node("annotator")
    annotator = Annotator()
    while (True):
        command = get_command() 
        if command[0] == "exit":
            return
        elif command[0] == "list":
            saved_msgs = annotator.get_saved_msgs()
            if (len(saved_msgs) == 0):
                print "No poses"
            else:
                print "Poses:"
                for name in saved_msgs:
                    print "  " + name
        elif command[0] == "save":
            annotator.save_pose(command[1])
        elif command[0] == "delete":
            result = annotator.delete_pose(command[1])
            if not result:
                print "No such pose '" + command[1] + "'" 
        elif command[0] == "goto":
            result = annotator.go_to(command[1])
            if not result:
                print "No such pose '" + command[1] + "'"
        elif command[0] == "read":
            result = annotator.get_msg(command[1])
            if not result:
                print "No such pose '" + command[1] + "'"
        elif command[0] == "help":
            print commands_msg
        else:
            print "Invalid command. Please try again"


if __name__ == "__main__":
    main()
