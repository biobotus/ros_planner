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
        # Colony selection parameters
        self.perimeter_min = 0
        self.perimeter_max = 0
        self.excentricity_min = 0
        self.excentricity_max = 0
        self.area_min = 0
        self.area_max = 0
        self.number_of_colony_d = 0
        self.picking = 0
        self.protocol =  0
        self.step = 0

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

        msg.picking = 0
        msg.protocol = x
        msg.protocol = y
        gripper_mod.gripper_move(backlight_mod,petri_mod,self.height,module_dic,self.steps)

        self.steps.append(bca_analysis(data, module_dic))

        return


    def _parse_autopick(self, var, module_dic):




        return

    def bca_analysis(self, data, module_dic):

        #args =  {"": data[1], "", data[2]}
        params = {"name": "analysis", "args": args}
        step_move = Step({"module_type": self.m_type, "params": params})

        return

    def bca_autopick(self, var, module_dic):

        #args =  {""}
        params = {"name": "analysis", "args": args}
        step_move = Step({"module_type": self.m_type, "params": params})

        return step_move
