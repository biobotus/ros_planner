#!/usr/bin/python

from deck.deck_module import Coordinate, DeckModule
from protocol.protocol import Protocol, Step, StepParameter

class BCAModule(DeckModule):
    def __init__(self, m_type, coord):
        super(BCAModule, self).__init__(m_type, coord)
        self.logger.info("BCA initialized")
        self.m_type = m_type
        self.mod_coord = self.get_mod_coordinate()
        self.x_buff = 0
        self.y_buff = 0
        # Constant used for simple pipette across the code

    def parse_json(self, json_instruction, module_dic):
        self.steps = []

        for instruction in json_instruction['groups']:
            if "repiquage" in instruction:
                self._parse_clap(instruction['repiquage'], module_dic)
            else:
                self.logger.error("unknown operation: {0}".format(instruction))
                pass
        return self.steps

    def _parse_repiquage(self, var, module_dic):


        return



