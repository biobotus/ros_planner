#/usr/bin/

from deck.deck_module import Coordinate, DeckModule, ModuleParam
from protocol.protocol import Step, StepParameter

class PipetteModule(DeckModule):

    def __init__(self, name, coord, m_type):
        super(PipetteModule, self).__init__(name, coord)
        #self.add_parameter(ModuleParam("volume", True, False))
        #self.add_parameter(ModuleParam("from", True, False))
        #self.add_parameter(ModuleParam("to", True, False))
        self.logger.info("Pipette initialized")
        self.m_type = m_type

    def parse_json(self, json_instruction, module_dic):
        steps = []

        for instruction in json_instruction:
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
                self.logger.error("unknown operation: {0}".format(instruction))
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
            * dispense
            * get up
            * go to the dump
            * eject tip
        :param trans_json: the json containing the information
        :param module_dict: the list of module involved in the protocol
        :return: a list of steps with their different parameters to complete
            the task
        """

        self.logger.info("parsing transfert instruction")



        # Coord initialized
        from_coord = self.parse_mod_coord(trans_json["from"], module_dic)
        to_coord = self.parse_mod_coord(trans_json["to"], module_dic)
        steps = []

        # Going to the source position
        from_coord.coord_z = 10
        steps.append(self.move_pos(from_coord))

        # Getting down
        from_coord.coord_z = 100
        steps.append(self.move_pos(from_coord))

        # aspirate
        steps.append(self.aspirate(trans_json["volume"],
                                    trans_json["aspirate_speed"]))

        # get Up
        from_coord.coord_z = 10
        steps.append(self.move_pos(from_coord))

        # got to the destination well
        to_coord.coord_z = 10
        steps.append(self.move_pos(to_coord))
        # getting down
        to_coord.coord_z = 100
        steps.append(self.move_pos(to_coord))

        # blow
        steps.append(self.dispense(trans_json["volume"],
                                    trans_json["dispense_speed"]))
        # get up
        to_coord.coord_z = 10;
        steps.append(self.move_pos(to_coord))

        steps.append(self.eject_tip("Large", module_dic))

        return steps

    def _parse_mix(self, mix_json):
        self.logger.info("parsing mix instruction")

    def aspirate(self, volume, speed):

        args = {"vol": float(volume), "speed": float(speed)}
        params = {"name": "manip", "args": args}
        step_move = Step({"module_type": self.m_type, "params": params})

        print(step_move)
        return step_move

    def dispense(self, volume, speed):

        args = {"vol": -float(volume), "speed": float(speed)}
        params = {"name": "manip", "args": args}
        step_move = Step({"module_type": self.m_type, "params": params})

        print(step_move)
        return step_move

    def move_pos(self, coord):

        args =  {"x": coord.coord_x, "y": coord.coord_y, "z": coord.coord_z}
        params = {"name": "pos", "args": args}
        step_move = Step({"module_type": self.m_type, "params": params})

        print(step_move)
        return step_move

    def eject_tip(self,tip_size, module_dic):
        steps=[]
        to_coord = Coordinate(0,0,0)
        # Get trash_bin from deck
        trash_mod = module_dic["trash"]
        dump_coord  = trash_mod.get_mod_coordinate()
        steps.append(self.move_pos(dump_coord))

        # TODO ajouter le step pour ejecter le tip
        # step down, move forward to the tooth
        if tip_size=="Large":
            to_coord.coord_z = 79 #mm
        elif tip_size=="Medium":
            to_coord.coord_z = 51 #mm
        elif tip_size=="Small":
            to_coord.coord_z = 46.5 #mm
        else:
            print("Error reading tip size")
        # Depends on tip size
        to_coord.coord_x = 10;
        steps.append(self.move_pos(to_coord)) # x&z
        # get up
        if tip_size=="Large":
            to_coord.coord_z = -30.09 #mm
        elif tip_size=="Medium":
            to_coord.coord_z = -20.29 #mm
        elif tip_size=="Small":
            to_coord.coord_z = -11.51 #mm
        steps.append(self.move_pos(to_coord))
        # ready to move away
        return to_coord

    def parse_mod_coord(self, dest_string, mod_dict):
        dest = dest_string.split("/")
        mod_name = dest[0]
        letter = dest[1][0]
        number = dest[1][1:]

        if mod_name in mod_dict:
            mod = mod_dict[mod_name]
            return mod.get_well_coordinate(ord(letter), int(number))
        else:
            print "ERROR no module to set coord"
            self.logger.error("attempt to access the coordinate of a module wich is not reference on the refs section of the json")
            #TODO raise an error
            return None
