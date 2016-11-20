#!/usr/bin/python

import logging
import pymongo
import rospy
from std_msgs.msg import Bool, String

from deck.deck_module import DeckManager
from deck.labware_module import Trash_bin, Small_Tip_Holder, Medium_Tip_Holder, \
                                Large_Tip_Holder, Centrifuge_Vial_Holder, \
                                Multiwell_Plate, Large_Container
from deck.pipette_module import PipetteModule
import protocol.protocol as protocol

class Planner():
    def __init__(self):
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz

        # ROS subscriptions
        self.subscriber = rospy.Subscriber('Start_Protocol', String, \
                                           self.callback_start_protocol)
        self.subscriber = rospy.Subscriber('Step_Done', Bool, \
                                           self.callback_done_step)

        # ROS publishments
        self.send_step = rospy.Publisher('New_Step', String, queue_size=10)
        self.module_manager = DeckManager()
        self.module_manager.add_tools()
        self.logger = logging.getLogger(__name__)

        self.step_complete = False

    def callback_start_protocol(self, data):
        prot = protocol.load_protocol_from_json(data.data, self.module_manager)
        print("Protocol loaded from JSON file:")
        print(data.data)

        for step in prot.steps:
            print(step)
            self.step_complete = False
            self.send_step.publish(str(step))
            while not self.step_complete:
                self.rate.sleep()

            print("Step complete!")

    def callback_done_step(self, data):
        if data.data == True:
            print("Done step true")
            self.step_complete = True
        else:
            rospy.signal_shutdown("Error: Done Step is false")
            return -1

    def listener(self):
        rospy.spin()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    try:
        p = Planner()
        p.listener()

    except rospy.ROSInterruptException as e:
        print(e)

