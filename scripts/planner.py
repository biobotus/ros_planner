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
        self.labware = {};
        self.i = 0

    def callback_input(self,data):

        self.name = data.name
        self.coord_x = data.coord_x
        self.coord_y = data.coord_y
        self.coord_z = data.coord_z
        self.labware["name"]=self.name
        self.labware["x_coord"]=self.coord_x
        self.labware["y_coord"]=self.coord_y
        self.labware["z_coord"]=self.coord_z
        #self.labware.append(RectContainerLabware(self.name))
        # = RectContainerLabware(self.name)

        print(self.labware)
        self.i=self.i+1
        for self.name in self.labware:
            print(self.labware["name"], self.coord_x)



    def listener(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        p = Planner()
        p.listener()

    except rospy.ROSInterruptException as e:
        print(e)
