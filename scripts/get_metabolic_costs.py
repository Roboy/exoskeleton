#!/usr/bin/env python

from roboy_simulation_msgs.srv import GetMetabCsv
import rospy
from os import listdir
import random


def get_metab_csv(req):
    possible_files = listdir("../data")
    return possible_files[random.randint(0, len(possible_files))]


def get_metab_server():
    rospy.init_node('get_metab_csv_server')
    s = rospy.Service('get_metab_csv', GetMetabCsv, get_metab_csv)
    print "Ready to return file paths."
    rospy.spin()


if __name__ == "__main__":
    get_metab_server()
