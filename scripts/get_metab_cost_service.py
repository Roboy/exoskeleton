#!/usr/bin/env python

from roboy_simulation_msgs.srv import GetMetabCsv
import rospy
from get_metabolic_costs import get_random_metab_csv_from_dir


def get_metab_csv(req):
    return get_random_metab_csv_from_dir("../data")


def get_metab_server():
    rospy.init_node('get_metab_csv_server')
    s = rospy.Service('get_metab_csv', GetMetabCsv, get_metab_csv)
    print "Ready to return file paths."
    rospy.spin()


if __name__ == "__main__":
    get_metab_server()
