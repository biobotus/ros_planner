#! /usr/bin/python

# Imports
import rospy
import numpy
from deck.deck_module import DeckManager
from deck.labware_module import Trash_bin
from deck.deck_module import DeckModule
from deck.deck_module import ModuleParam
from deck.deck_module import Coordinate
from deck.pipette_module import PipetteModule
from deck.labware_module import Rect4Container
import logging
from protocol.protocol import *
from ros_planner.msg import CoordinateMsgs
from deck.tac_module import TacModule
from std_msgs.msg import Bool, String

class Planner():

    def __init__(self):
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz

        # ROS subscriptions
        self.subscriber = rospy.Subscriber('Deck_Item', CoordinateMsgs, self.callback_input)
        self.subscriber = rospy.Subscriber('Start_Protocol', String, self.callback_start_protocol)
        self.subscriber = rospy.Subscriber('Done_Step', Bool, self.callback_done_step)

        # ROS publishments
        # TODO - Change message format
        self.send_step = rospy.Publisher('Protocol_Step', String, queue_size=10)

        self.modules = DeckManager()
        self.logger = logging.getLogger(__name__)

        self.step_complete = False

    def callback_start_protocol(self, data):
        prot = load_protocol_from_json_file(data.data, self.modules)
        print("Protocol loaded from JSON file:")
        print(data.data)

        for step in prot.steps:
            self.step_complete = False
            self.send_step.publish(step)

            while not self.step_complete:
                self.rate.sleep()

            print("Step complete!")


    def callback_done_step(self, data):
        if data.data:
            self.step_complete = True

    def callback_input(self, data):
        try:
            parameters = [self, data.m_name, data.m_id, Coordinate(data.coord_x, data.coord_y ,data.coord_z)]
            getattr(self.__class__, 'add_{0}'.format(data.m_type))(*parameters)
        except AttributeError as e:
            print(e)

    def add_rect4container(self,m_name,m_id,coord):
        self.logger.info("Add rectangular 4 col container")
        rect_4_labware = Rect4Container(m_name, coord)
        self.modules.add_module(rect_4_labware, m_id)

    def add_tac(self,m_name,m_id,coord):
        self.logger.info("Add tac module")
        tac_module = TacModule(m_name, coord)
        self.modules.add_module(tac_module, m_id)

    def add_pipette(self,m_name,m_id,coord):
        self.logger.info("Add pipette module")
        pipette_module = PipetteModule(m_name,coord)
        self.modules.add_module(pipette_module,m_id)

    def add_trash(self, m_name, m_id, coord):
        self.logger.info("Ass thrash module")
        trash_module = Trash_bin(m_name, coord)
        self.modules.add_module(trash_module, m_id)

    def listener(self):
        rospy.spin()


if __name__ == "__main__":

    logging.basicConfig(level=logging.INFO)

    try:
        p = Planner()
        p.listener()

    except rospy.ROSInterruptException as e:
        print(e)
