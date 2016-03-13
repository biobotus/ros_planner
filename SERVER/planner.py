#! /usr/bin/python

# Imports
import rospy
from deck.deck_module import DeckManager
from deck.deck_module import DeckModule
from deck.deck_module import ModuleParam
from deck.deck_module import Coordinate
from deck.pipette_module import PipetteModule
from deck.labware_module import RectContainerLabware
import logging
from protocol.protocol import *

class Planner():

    def __init__(self):

        logging.basicConfig(level=logging.INFO)
        module_mgr = DeckManager()

        water_labware = RectContainerLabware("water")
        dye_labware = RectContainerLabware("dye")
        samples_labware = RectContainerLabware("samples")

        tac_module = DeckModule("tac")
        tac_module.add_parameter(ModuleParam(name="temperature", p_in=True,
                                            p_out=True, vmax=40, vmin=4))
        tac_module.add_parameter(ModuleParam(name="density", p_in=False, p_out=True))
        tac_module.add_parameter(ModuleParam(name="spin", p_in=True,
                                            p_out=False, vmax=100, vmin=0))

        tac_module.set_well_layout(1, 1, Coordinate(10, 10, 0), Coordinate(0, 0, 0))
        pipette_module = PipetteModule("pipette")

        module_mgr.add_module(water_labware, "ct149x8mea3j")
        module_mgr.add_module(dye_labware, "ct13zjq79whe")
        module_mgr.add_module(samples_labware, "ct3b245kx34l")

        module_mgr.add_module(tac_module, tac_id)
        module_mgr.add_module(pipette_module, pipette_id)

        #loadProtocolFromJson('tac.json')
        prot = load_protocol_from_json_file('./json/tac.json', module_mgr)
        #protocol = Protocol("test1")
        print prot.steps

    def callback_input(self,data):

        self.info = data.data
        print(self.info)


    def listener(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        p = Planner()
        p.listener()
        
    except rospy.ROSInterruptException as e:
        print(e)
