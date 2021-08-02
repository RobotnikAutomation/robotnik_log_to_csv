#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from log_to_csv import LogToCsv


def main():

    rospy.init_node("log_to_csv_node")

    rc_node = LogToCsv()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
