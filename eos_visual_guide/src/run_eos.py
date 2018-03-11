#!/usr/bin/env python
import rospy
from eos_visual_guide import EosController


def main():
    rospy.init_node('run_eos')
    c = EosController()
    c.run()


if __name__ == '__main__':
    main()
