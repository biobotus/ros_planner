__author__ = 'Do'
from deck.deck_module import DeckModule
from deck.deck_module import ModuleParam
from deck.deck_module import Coordinate

from protocol.protocol import Step, StepParameter

import re


class PipetteModule(DeckModule):

    def __init__(self, name, coor=Coordinate(0, 0, 0)):
        super(PipetteModule, self).__init__(name, coor)
        self.add_parameter(ModuleParam("volume", True, False))
        self.add_parameter(ModuleParam("from", True, False))
        self.add_parameter(ModuleParam("to", True, False))

    def parse_json(self, json_instruction, module_dic):
        steps = []
        for instruction in json_instruction['groups']:
            if "distribute" in instruction:
                #TODO raise an exception, this operation won't be possible with the mechaPipette
                self.logger.error("distribute operation is not possible with the mecha pipette for now")
                
            elif "consolidate" in instruction:
                #TODO raise an exception, this operation won't be possible with the mechaPipette
                self.logger.error("consolidate operation is not possible with the mecha pipette for now")
                
            elif "transfert" in instruction:
                steps = self._parse_transfert(instruction['transfert'], module_dic)
            elif "mix" in instruction:
                #TODO est possible avec la mechaPipette?
                steps = self._parse_mix(instruction['mix'])
            else:
                #TODO raise an exception instruction not known
                self.logger.error("unknown operation : %s", instruction)
                pass
        return steps


    def _parse_transfert(self, trans_json, module_dic):
        """
        A transfert is an operation which transfert from one well to another a
        certain amount of liquid
        There is 8 step to do so :
            * go to the source well position
            * get down
            * aspirate
            * get up
            * go to the destination well
            * get down
            * blow
            * get up
            * go to the dump
            * eject tip 
        :param trans_json: the json containing the information
        :param module_dict: the list of module involved in the protocol 
        :return: a list of step with their different parameter to complete
            the task
        """

        self.logger.info("parsing transfert instruction")
     
        from_coor = self.parse_mod_coor(trans_json["from"], module_dic)        
        to_coor = self.parse_mod_coor(trans_json["to"], module_dic)

        #TODO trouver un moyen de gerer la poubelle
        dump_coor = Coordinate(5, 5, 10)
        
        steps = []
        # Going to the source position
        from_coor.coor_z = 10
        steps.append(self.get_go_to_step(from_coor))

        # Getting down
        # TODO we need to retrieve the height
        from_coor.coor_z = 100
        steps.append(self.get_go_to_step(from_coor))

        # aspirate
        # TODO ajouter le step pour aspirer 
        # get Up
        from_coor.coor_z = 10
        steps.append(self.get_go_to_step(from_coor))
        
        # got to the destination well
        to_coor.coor_z = 10
        steps.append(self.get_go_to_step(to_coor))
        # getting down
        to_coor.coor_z = 100
        steps.append(self.get_go_to_step(to_coor))
        
        # blow
        # TODO ajouter le step pour ejecter  
        # get up
        to_coor.coor_z = 10;
        steps.append(self.get_go_to_step(to_coor))
        
        # go to the dump
        steps.append(self.get_go_to_step(dump_coor))
        
        # eject the tip
        # TODO ajouter le step pour ejecter le tip
        print steps
        
        return steps
        
    def _parse_mix(self, mix_json):
        self.logger.info("parsing mix instruction")

    def get_go_to_step(self, coor):
        coor_to = Coordinate(coor_x=coor.coor_x, coor_y=coor.coor_y, coor_z=0)
        stop_condition = StepParameter(module=self,
                                       name="position",
                                       value=coor_to)

        step_move = Step(stop_condition)
        step_move.add_parameter(StepParameter(module=self,
                                              name="destination",
                                              value=coor_to))
        return step_move

    def get_down_steps(self, height):
        stop_condition = StepParameter(module=self,
                                       name="position",
                                       value=coor_to)

        step_move = Step(stop_condition)
        step_move.add_parameter(StepParameter(module=self,
                                              name="destination",
                                              value=coor_to))
        return step_move

    def parse_mod_coor(self, dest_string, mod_dict):
        dest = dest_string.split("/")
        well = re.split('(\d+)', dest[1])
        mod_name = dest[0]
        letter = well[0]
        number = well[1]
        
        if mod_name in mod_dict: 
            mod = mod_dict[mod_name]
            return mod.get_well_coordinate(ord(letter), int(number))
        else:
            self.logger.error("attempt to access the coordinate of a module wich is not reference on the refs section of the json")
            #TODO raise an error

