#! /usr/bin/env python
import rospy
from rda_core import rda_core


if __name__ == '__main__':
    rda = rda_core()
    rda.control()
