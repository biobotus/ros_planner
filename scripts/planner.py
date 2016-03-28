#! /usr/bin/python

# Imports
import rospy
import numpy
from deck.deck_module import DeckManager
from deck.deck_module import DeckModule
from deck.deck_module import ModuleParam
from deck.deck_module import Coordinate
from deck.pipette_module import PipetteModule
from deck.labware_module import RectContainerLabware
import logging
from protocol.protocol import *
from ros_planner.msg import CoordinateMsgs
from deck.tac_module import TacModule

class Planner():

    def __init__(self):
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.subscriber = rospy.Subscriber('modules_position', CoordinateMsgs, self.callback_input)
        self.modules = DeckManager()
        self.logger = logging.getLogger(__name__)

    def callback_start_protocol(self, data):

        prot = load_protocol_from_json_file('src/ros_planner/json/tac.json', self.modules)

    def callback_input(self,data):

        if data.run:
            print "Loading protocol"
            prot = load_protocol_from_json_file("src/ros_planner/json/tac.json", self.modules)

        else:

            try:
                getattr(self.__class__, 'add_{0}'.format(data.type))(self, data.name,data.m_id, Coordinate(data.coord_x,data.coord_y ,data.coord_z ))
            except AttributeError as e:
                print(e)

            self.modules.list_module()

    def add_tac(self,name,m_id,coord):
        self.logger.info("Add tac module")
        tac_module = TacModule(name,coord)
        self.modules.add_module(tac_module,m_id)

    def add_pipette(self,name,m_id,coord):
        self.logger.info("Add pipette module")
        pipette_module = PipetteModule(name,coord)
        self.modules.add_module(pipette_module,m_id)

    def listener(self):
        rospy.spin()


if __name__ == "__main__":

    logging.basicConfig(level=logging.INFO)

    try:
        p = Planner()
        p.listener()

    except rospy.ROSInterruptException as e:
        print(e)
