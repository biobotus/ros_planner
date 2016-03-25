#! /usr/bin/python

# Imports
import rospy
import numpy
#from deck.deck_module import DeckManager
#from deck.deck_module import DeckModule
#from deck.deck_module import ModuleParam
#from deck.deck_module import Coordinate
#from deck.pipette_module import PipetteModule
from deck.labware_module import RectContainerLabware
#import logging
#from protocol.protocol import *
from ros_planner.msg import CoordinateMsgs

class Planner():

    def __init__(self):
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.subscriber = rospy.Subscriber('modules_position', CoordinateMsgs, self.callback_input)
        self.module = DeckManager()


    def callback_input(self,data):

        self.module.add_module(data.id)
        self.module.get_module(data.id)
        try:
            getattr(self.__class__, 'add_{0}'.format(data.type))(self, data.id)
        except AttributeError as e:
            print(e)

    def add_tac(self,id):
        print "BABOO"

    def listener(self):
        rospy.spin()



if __name__ == "__main__":
    try:
        p = Planner()
        p.listener()

    except rospy.ROSInterruptException as e:
        print(e)
