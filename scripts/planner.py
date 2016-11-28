#!/usr/bin/python

import logging
import pymongo
import rospy
import time
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

        # Protocol Stats Collection of BioBot Database
        self.stats = pymongo.MongoClient()['biobot']['stats']

        # ROS subscriptions
        self.subscriber_prot = rospy.Subscriber('Start_Protocol', String, \
                                           self.callback_start_protocol)
        self.subscriber = rospy.Subscriber('Step_Done', Bool, \
                                           self.callback_done_step)
        self.subscriber_mapping = rospy.Subscriber('Start_Mapping', Bool, \
                                           self.callback_start_mapping)

        self.subscriber_pause = rospy.Subscriber('Pause', Bool, \
                                           self.callback_pause)
        print('sfdg')
        # ROS publishments
        self.send_step = rospy.Publisher('New_Step', String, queue_size=10)
        self.send_status = rospy.Publisher('BioBot_Status', String, queue_size=10)

        self.module_manager = DeckManager()
        self.module_manager.add_tools()
        self.logger = logging.getLogger(__name__)

        self.step_complete = False
        self.doing_mapping = False
        self.doing_protocol = False
        self.paused = False

    def callback_pause(self, data):
        if data.data == True:
            print('PAUSED BIOBOT')
            self.paused = True
        else:
            print('UNPAUSED BIOBOT')
            self.paused = False

    def callback_start_mapping(self, data):
        while self.doing_protocol:
            self.rate.sleep()

        self.doing_mapping = True

        if data.data:
            print('Starting Mapping')
            prot = protocol.mapping_3d_protocol(self.module_manager)

            for high_level_step in prot.steps:
                for step in high_level_step:
                    while self.paused:
                        self.rate.sleep()
                    print(step)
                    self.step_complete = False
                    self.send_step.publish(str(step))
                    while not self.step_complete:
                        self.rate.sleep()
                    print("Step complete!")

        self.doing_mapping = False

    def callback_start_protocol(self, data):
        while self.doing_mapping:
            self.rate.sleep()

        self.doing_protocol = True
        print("Protocol loaded from JSON file:")
        print(data.data)

        prot = protocol.load_protocol_from_json(data.data, self.module_manager)
        self.stats.insert_one({'id': prot.name, 'name': prot.data['name'], \
                               'description': prot.data['description'], \
                               'nb_steps': len(prot.steps), 'start': time.time(), \
                               'operator': prot.data['operator']})

        number = 1
        for high_level_step in prot.steps:
            prot.db.steps.update_one({'number': number}, {'$set': {'start': time.time()}})
            for step in high_level_step:
                while self.paused:
                    self.rate.sleep()
                print(step)
                self.step_complete = False
                self.send_step.publish(str(step))
                while not self.step_complete:
                    self.rate.sleep()
                print("Step complete!")

            prot.db.steps.update_one({'number': number}, {'$set': {'end': time.time()}})
            number += 1

        self.stats.update_one({'id': prot.name}, {'$set': {'end': time.time()}})
        self.doing_protocol = False

    def callback_done_step(self, data):
        if data.data == True:
            print("Done step true")
            self.step_complete = True
        else:
            rospy.signal_shutdown("Error: Done Step is false")
            return -1

    def listener(self):
        self.status_rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.doing_protocol and self.paused:
                self.send_status.publish('paused (doing a biological protocol)')
            elif self.doing_mapping and self.paused:
                self.send_status.publish('paused (mapping the deck)')
            elif self.doing_protocol:
                self.send_status.publish('doing a biological protocol')
            elif self.doing_mapping:
                self.send_status.publish('mapping the deck')
            elif self.paused:
                self.send_status.publish('paused')
            else:
                self.send_status.publish('idle')
            self.status_rate.sleep()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    try:
        p = Planner()
        p.listener()

    except rospy.ROSInterruptException as e:
        print(e)

