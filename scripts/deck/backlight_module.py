#!/usr/bin/python

from deck.deck_module import Coordinate, DeckModule
from protocol.protocol import Protocol, Step

class BackLightModule(DeckModule):
    def __init__(self, m_type, coord):
        super(BackLightModule, self).__init__(m_type, coord)
        self.logger.info("BCA initialized")
        self.m_type = m_type
        self.mod_coord = self.get_mod_coordinate()
        self.x_buff = 0
        self.y_buff = 0
        # Constant used for simple pipette across the code

    def parse_json(self, json_instruction, module_dic):
        self.steps = []

        self.petri_mod = module_dic[json_instruction['name']]
        gripper_mod = module_dic["gripper"]

        for instruction in json_instruction['groups']:

            if "analyze" in instructions:
                self._parse_analyze(instruction['analyze'], module_dic)
            elif "autopick" in instruction:
                self._parse_autopick(instruction['autopick'], module_dic)
            else:
                self.logger.error("unknown operation: {0}".format(instruction))
                pass
        return self.steps

    def _parse_analyze(self,var, module_dic):

        gripper_mod.gripper_move(backlight_mod,petri_mod,self.height,module_dic,self.steps)

        return


    def _parse_autopick(self, var, module_dic):


        return



