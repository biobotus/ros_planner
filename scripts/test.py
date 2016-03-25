#!/usr/bin/python

from deck.deck_module import DeckManager
from deck.deck_module import DeckModule
from deck.deck_module import ModuleParam
from deck.deck_module import Coordinate
from deck.pipette_module import PipetteModule
from deck.labware_module import RectContainerLabware
import logging
import planner_input

from protocol.protocol import *

if __name__ == "__main__":

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
    #construction des steps
    #step1 = Step(StepParameter(TacModule, "density", 80))
    #step1.add_parameter(StepParameter(TacModule, "temperature", 37))
    #step1.add_parameter(StepParameter(TacModule, "agitation", 50))


    #step2 = Step(StepParameter(TacModule, "time", 20))
    #step2.add_parameter(StepParameter(TacModule,"temperature", 4))
    #step2.add_parameter(StepParameter(TacModule,"agitation", 0))


    #Ajout des steps a la Protocol
    #protocol.addStep(step1)
    #protocol.addStep(step2)

	# debut de la Protocol
    #protocol.start()

    # on recoit du module le message de debut du step
    #protocol.onStepStartedReceived(TacModule)

    # on recoit plusieur message de parametre des modules
    #protocol.onParamReceived(TacModule, "temperature", 10)
    #time.sleep(15)
    #protocol.onParamReceived(TacModule, "temperature", 37)
    #time.sleep(15)
    #protocol.onParamReceived(TacModule, "density", 80)
    #time.sleep(15)
    #on a recut le message density 80 qui etait le stop parameter
    #on recoit maintenant le message comme quoi le step suivant a debute
    #protocol.onStepStartedReceived(TacModule)

    # on recoit plusieur message de parametre des modules
    # mais le stop parameter est temporelle
    #protocol.onParamReceived(TacModule, "temperature", 10)
    #time.sleep(15)
    #protocol.onParamReceived(TacModule, "temperature", 37)
    #time.sleep(15)
    #protocol.onParamReceived(TacModule, "density", 80)
