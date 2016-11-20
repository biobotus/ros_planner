#!/usr/bin/python

from deck.deck_module import Coordinate, DeckModule
from protocol.protocol import Step, StepParameter

class GripperModule(DeckModule):

    def __init__(self, name, coord, m_type):
        super(PipetteModule, self).__init__(name, coord)
        self.logger.info("Gripper initialized")
        self.m_type = m_type

        self.xL=[0]
        self.yL=[0]
        self.xM=[0]
        self.yM=[0]
        self.xS=[0]
        self.yS=[0]

    def parse_json(self, json_instruction, module_dic):
        steps = []

        for instruction in json_instruction:
            if "maneuver" in instruction:
                steps = self._parse_maneuver(instruction['maneuver'], module_dic)
            #elif "transfert" in instruction:
            #    steps = self._parse_transfert(instruction['transfert'], module_dic)
            else:
                #TODO raise an exception instruction not known
                self.logger.error("unknown operation: {0}".format(instruction))
                pass
        return steps


    def _parse_maneuver(self, trans_json, module_dic):
        """
        A maneuver is an operation which move an object from point A to point B
        There is 8 step to do so :
            * go to object A
            * open the gripper
            * go to a relevant z position (get down)
            * close the gripper
            * move up
            * (rotate/other weird move)
            * go to position B
            * get down
            * open the gripper (0°)
            * get up
        :param trans_json: the json containing the information
        :param module_dict: the list of module involved in the protocol
        :return: a list of steps with their different parameters to complete
            the task
        """
        import math

        # All python functions are in rads
        # gripper data :
        z = 18
        x = 110
        h = x*cos(angle)
        110*math.cos(math.radians(63))






        self.logger.info("parsing transfert instruction")



        # Coord initialized
        from_coord = self.parse_mod_coord(trans_json["from"], module_dic)
        to_coord = self.parse_mod_coord(trans_json["to"], module_dic)
        steps = []

        # Going to the source position
        steps.append(self.move_pos(from_coord, module_dic))



        def calculate_angle(self,module_dic):
            #to be written
        return steps
