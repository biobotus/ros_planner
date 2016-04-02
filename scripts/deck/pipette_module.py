#/usr/bin/

from deck.deck_module import Coordinate, DeckModule, ModuleParam
from protocol.protocol import Step, StepParameter

class PipetteModule(DeckModule):

    def __init__(self, name, coord):
        super(PipetteModule, self).__init__(name, coord)
        self.add_parameter(ModuleParam("volume", True, False))
        self.add_parameter(ModuleParam("from", True, False))
        self.add_parameter(ModuleParam("to", True, False))
        self.logger.info("Pipette initialized")

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

        # Get dump from deck
        trash_mod = module_dic["trash"]
        dump_coord  = trash_mod.get_mod_coordinate()

        # Get coord from and to
        from_coord = self.parse_mod_coord(trans_json["from"], module_dic)
        from_aspirate = self.parse_mod_aspirate(trans_json["volume"],
                                                trans_json["aspirate_speed"], module_dic)

        to_coord = self.parse_mod_coord(trans_json["to"], module_dic)

        steps = []

        # Going to the source position
        print("Coord pos")
        print(trans_json["to"])
        print("aspirate param : ")
        print(trans_json["volume"])
        from_coord.coord_z = 10
        steps.append(self.get_go_to_step(from_coord))

        # Getting down
        print(from_coord)
        from_coord.coord_z = 100
        print(from_coord)
        steps.append(self.get_go_to_step(from_coord))

        # aspirate
        steps.append(self.aspirate(trans_json["volume"],
                                    trans_json["aspirate_speed"]))

        # get Up
        from_coord.coord_z = 10
        steps.append(self.get_go_to_step(from_coord))

        # got to the destination well
        to_coord.coord_z = 10
        steps.append(self.get_go_to_step(to_coord))
        # getting down
        to_coord.coord_z = 100
        steps.append(self.get_go_to_step(to_coord))

        # blow
        steps.append(self.dispense(trans_json["volume"],
                                    trans_json["dispense_speed"]))
        # get up
        to_coord.coord_z = 10;
        steps.append(self.get_go_to_step(to_coord))

        # go to the dump
        steps.append(self.get_go_to_step(dump_coord))

        # eject the tip
        # TODO ajouter le step pour ejecter le tip

        return steps

    def _parse_mix(self, mix_json):
        self.logger.info("parsing mix instruction")

    def aspirate(self, speed, volume):
        stop_condition = StepParameter(module=self,
                                        name="volume",
                                        value=volume)
        step_move = Step(stop_condition)
        step_move.add_parameter(StepParameter(module=self,
                                        name="speed",
                                        value=speed))

        print(step_move)
        return step_move

    def dispense(self, speed, volume):
        stop_condition = StepParameter(module=self,
                                        name="volume",
                                        value=-int(volume))
        step_move = Step(stop_condition)
        step_move.add_parameter(StepParameter(module=self,
                                        name="speed",
                                        value=speed))


        print(step_move)
        return step_move

    def get_go_to_step(self, coord):
        coord_to = Coordinate(coord_x=coord.coord_x, coord_y=coord.coord_y, coord_z=coord.coord_z)
        stop_condition = StepParameter(module=self,
                                       name="position",
                                       value=coord_to)

        step_move = Step(stop_condition)
        step_move.add_parameter(StepParameter(module=self,
                                              name="destination",
                                              value=coord_to))
        print(step_move)
        return step_move

    def get_down_steps(self, height):
        stop_condition = StepParameter(module=self,
                                       name="position",
                                       value=coord_to)

        step_move = Step(stop_condition)
        step_move.add_parameter(StepParameter(module=self,
                                              name="destination",
                                              value=coord_to))
        print(step_move)
        return step_move

    def parse_mod_aspirate(self, volume, speed, mod_dict):
        print("ASPIRATE!")
        return[volume, speed]


    def parse_mod_coord(self, dest_string, mod_dict):
        dest = dest_string.split("/")
        mod_name = dest[0]
        letter = dest[1][0]
        number = dest[1][1:]
        print(dest)

        if mod_name in mod_dict:
            mod = mod_dict[mod_name]
            print(mod)
            print(letter)
            print(number)
            print(mod.get_well_coordinate(ord(letter), int(number)))
            return mod.get_well_coordinate(ord(letter), int(number))
        else:
            print "ERROR no module to set coord"
            self.logger.error("attempt to access the coordinate of a module wich is not reference on the refs section of the json")
            #TODO raise an error
            return None
