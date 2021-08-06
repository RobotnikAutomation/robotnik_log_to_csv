#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rcomponent.rcomponent import *

# Insert here general imports:
import csv
import os
import re

# Insert here msg and srv imports:
from std_msgs.msg import String
from robotnik_msgs.msg import StringStamped

from std_srvs.srv import Trigger, TriggerResponse


class LogToCsv(RComponent):
    """
    Node to save the log topic into a .csv file.
    """

    def __init__(self):

        RComponent.__init__(self)

    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        self.log_topic_name = rospy.get_param(
            '~log_topic', 'robotnik_hmi/log')
        self.csv_folder_path = rospy.get_param(
            '~csv_folder_path', '/home/robot/logs')
        self.max_file_number = rospy.get_param(
            '~max_file_number', 10)

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        # Publisher
        self.data_pub = rospy.Publisher(
            '~data', String, queue_size=10)
        self.data_stamped_pub = rospy.Publisher(
            '~data_stamped', StringStamped, queue_size=10)

        # Subscriber
        self.log_sub = rospy.Subscriber(
            self.log_topic_name, String, self.log_sub_cb)
        RComponent.add_topics_health(self, self.log_sub, topic_id='log_sub', timeout=1.0, required=False)

        return 0

    def init_state(self):
        self.data = String()
        self.fieldnames = ["Time", "Type", "Log information"]

        return RComponent.init_state(self)

    def ready_state(self):
        """Actions performed in ready state"""

        # Check topic health

        if(self.check_topics_health() == False):
            self.switch_to_state(State.EMERGENCY_STATE)
            return RComponent.ready_state(self)

        # Publish topic with data

        data_stamped = StringStamped()
        data_stamped.header.stamp = rospy.Time.now()
        data_stamped.string = self.data.data

        self.data_pub.publish(self.data)
        self.data_stamped_pub.publish(data_stamped)

        return RComponent.ready_state(self)

    def emergency_state(self):
        if(self.check_topics_health() == True):
            self.switch_to_state(State.READY_STATE)

    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)

    def create_file(self, file):
        csv_file = open(file, 'w')
        writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames)
        writer.writeheader()

    def write_file(self, file, msg_time, msg_type, msg_data):
        csv_file = open(file, 'a')
        writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames)
        my_dic = {
            self.fieldnames[0]: msg_time,
            self.fieldnames[1]: msg_type,
            self.fieldnames[2]: msg_data
        }
        writer.writerow(my_dic)
        csv_file.close()

    def atoi(self, text):
        return int(text) if text.isdigit() else text

    def natural_keys(self, text):
        date = re.split(r'(\d+)', text)
        date[1], date[5] = date[5], date[1]
        return [ self.atoi(c) for c in date]

    def log_sub_cb(self, msg):
        # Get msg information
        try:
            msg_type, msg_date, msg_time, msg_data = msg.data.split(" ", 3)
        except ValueError:
            rospy.logerr("LogToCsv::log_sub_cb: Error on log format.")
            return
        msg_date = msg_date.replace("/","-")
        file = self.csv_folder_path + "/" + msg_date + ".csv"

        # Create the file if it doesn't already exist
        if not os.path.isfile(file):
            self.create_file(file)
            # Delete older files if max number of files reached
            onlyfiles = [f for f in os.listdir(self.csv_folder_path)]
            onlyfiles = sorted(onlyfiles, key=self.natural_keys, reverse=True)
            while (len(onlyfiles) > self.max_file_number):
                os.remove(self.csv_folder_path + "/" + onlyfiles.pop())

        # Save the log msg into the file
        self.write_file(file, msg_time, msg_type, msg_data)

        self.tick_topics_health('log_sub')
