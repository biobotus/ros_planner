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
        """
        Planner initialisation
        ROS topics init
        Flags init
        """

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
        self.subscriber_calib_2d = rospy.Subscriber('Pixel_Size', Bool, \
                                           self.callback_calibration_2d)

        self.subscriber_pause = rospy.Subscriber('Pause', Bool, \
                                           self.callback_pause)
        self.subsciber_clear_tip = rospy.Subscriber('Reset_Tips', Bool, \
                                            self.callback_reset_tips)
        # ROS publishments
        self.send_step = rospy.Publisher('New_Step', String, queue_size=10)
        self.send_status = rospy.Publisher('BioBot_Status', String, queue_size=10)

        self.module_manager = DeckManager()
        self.module_manager.add_tools()
        self.logger = logging.getLogger(__name__)

        self.step_complete = False
        self.busy = False
        self.paused = False
        self.current_action = 'idle'
        self.labware = []


    def callback_reset_tips(self, data):
        """
        Reset tips
        Sent by the web, this function reset the tip holders to the original state
        """

        for lab in self.labware:
            if lab['type'] == 'medium_tip_holder' or lab['type'] == 'small_tip_holder' or lab['type'] == 'large_tip_holder':
                mod = self.module_manager.modules[lab['name']]
                mod.reset_tips()
        return

    def callback_pause(self, data):
        """
        Pause
        Sent by the web, this function hold planner from sending another step to coordinator.
        When a pause is sent, coordinator finish the last step before pausing the platform.
        The platform can be unpaused by the web.
        """

        if data.data == True:
            print('PAUSED BIOBOT')
            self.paused = True
        else:
            print('UNPAUSED BIOBOT')
            self.paused = False

    def callback_start_mapping(self, data):

        """
        Start mapping
        Sent by the web, this function start a mapping protocol.
        The platform will move the head to scan the platform deck.
        9 positions are used to map the whole deck in a 3 by 3 layout
        """


        if not data.data:
            return

        while self.busy:
            self.rate.sleep()

        self.busy = True
        self.action = 'mapping the deck'

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

        self.busy = False

    def callback_calibration_2d(self, data):
        """
        This function is not yet implemented
        """

        if not data.data:
            return

        while self.busy:
            self.rate.sleep()

        self.busy = True
        self.action = 'calibrating 2d camera'

        print('Starting 2D camera calibration')
        prot = protocol.calibrate_2d_cam_protocol(self.module_manager)

        for high_level_step in prot.steps:
            for step in high_level_step:
                while self.paused:
                    self.rate.sleep()
                self.step_complete = False
                self.send_step.publish(str(step))
                while not self.step_complete:
                    self.rate.sleep()
                print("Step complete!")

        self.busy = False

    def callback_start_protocol(self, data):

        """
        Protocol
        Sent by the web, this function start a biological protocol formated by the web.
        Planner create all the steps the acheive a protocol before sending a first step to coordinator.
        After each step planner wait for a Done_Step from coordinator before sending another step.
        """

        while self.busy:
            self.rate.sleep()

        self.busy = True
        self.action = 'doing a biological protocol'
        print("Protocol loaded from JSON file:")
        print(data.data)

        prot, self.labware = protocol.load_protocol_from_json(data.data, self.module_manager)

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
        self.busy = False

    def callback_done_step(self, data):
        """
        Done step
        When a step is done, this function change a flag to send a new step
        """
        if data.data == True:
            print("Done step true")
            self.step_complete = True
        else:
            rospy.signal_shutdown("Error: Done Step is false")
            return -1

    def listener(self):
        """
        ROS callback listener
        """
        self.status_rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if not self.busy:
                status = 'idle'
            self.send_status.publish("{}{}".format(self.action if self.busy else 'idle', \
                                                   ' (paused)' if self.paused else ''))
            self.status_rate.sleep()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    try:
        p = Planner()
        p.listener()

    except rospy.ROSInterruptException as e:
        print(e)

