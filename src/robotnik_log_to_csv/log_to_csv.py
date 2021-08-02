#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rcomponent.rcomponent import *

# Insert here general imports:
import csv

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

    def log_sub_cb(self, msg):
        rospy.logwarn("Received msg: " + msg.data)
        # TODO Save on a .csv file into a parametrized folder (using "n" days limit)
        self.tick_topics_health('log_sub')
