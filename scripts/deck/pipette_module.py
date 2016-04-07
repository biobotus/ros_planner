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

        self.xL=[0]
        self.yL=[0]
        self.xM=[0]
        self.yM=[0]
        self.xS=[0]
        self.yS=[0]

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
            * get the tip
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
        # Get tip from tip holder module
        steps = self.get_tip("Small", module_dic, steps)

        # Going to the source position
        from_coord.coord_z = 10
        steps.append(self.move_pos(from_coord, module_dic))

        # Getting down
        from_coord.coord_z = 100
        steps.append(self.move_pos(from_coord, module_dic))

        # aspirate
        steps.append(self.aspirate(trans_json["volume"],
                                    trans_json["aspirate_speed"]))

        # get Up
        from_coord.coord_z = 10
        steps.append(self.move_pos(from_coord, module_dic))

        # got to the destination well
        to_coord.coord_z = 10
        steps.append(self.move_pos(to_coord, module_dic))
        # getting down
        to_coord.coord_z = 100
        steps.append(self.move_pos(to_coord, module_dic))
        # blow
        steps.append(self.dispense(trans_json["volume"],
                                    trans_json["dispense_speed"]))
        # get up
        to_coord.coord_z = 10;
        steps.append(self.move_pos(to_coord, module_dic))

        steps = self.eject_tip("Large", module_dic, steps)
        return steps

    def _parse_mix(self, mix_json):
        self.logger.info("parsing mix instruction")

    def aspirate(self, volume, speed):

        args = {"vol": float(volume), "speed": float(speed)}
        params = {"name": "manip", "args": args}
        step_move = Step({"module_type": self.m_type, "params": params})

        #print(step_move)
        return step_move

    def dispense(self, volume, speed):

        args = {"vol": -float(volume), "speed": float(speed)}
        params = {"name": "manip", "args": args}
        step_move = Step({"module_type": self.m_type, "params": params})

        #print(step_move)
        return step_move

    def move_pos(self, coord, module_dic):

        mod = module_dic["pipette"]
        mod_coord = mod.get_mod_coordinate()
        coord.coord_x = coord.coord_x-mod_coord.coord_x
        coord.coord_y = coord.coord_y-mod_coord.coord_y
        coord.coord_z = coord.coord_z-mod_coord.coord_z

        args =  {"x": coord.coord_x, "y": coord.coord_y, "z": coord.coord_z}
        params = {"name": "pos", "args": args}
        step_move = Step({"module_type": self.m_type, "params": params})

        #print(step_move)
        return step_move

    def get_tip(self, tip_size, module_dic, steps):


        to_coord = Coordinate(0,0,0)
            # Get tip holder from deck
        if tip_size=="Large":
            tip_mod = module_dic["large_tip_holder"]
#            to_coord = tip_mod.get_mod_coordinate()
            if self.xL[0]>7:
                self.yL[0]=self.yL[0]+1
                self.xL[0]=0
            to_coord = tip_mod.get_well_coordinate(self.xL[0],self.yL[0])
            #to_coord.coord_x = to_coord.coord_x + 7.286 + xL[0]*1.55
            #to_coord.coord_y = to_coord.coord_y + 7.286 + yL[0]*1.52
            steps.append(self.move_pos(to_coord, module_dic)) # move over first well
            self.xL[0]=self.xL[0]+1
            to_coord.coord_z = 30.09 #mm
            steps.append(self.move_pos(to_coord, module_dic)) # move over first well

        elif tip_size=="Medium":
            tip_mod = module_dic["medium_tip_holder"]
            #to_coord = tip_mod.get_mod_coordinate()
            if self.xM[0]>7:
                self.yM[0]=self.yM[0]+1
                self.xM[0]=0
            to_coord = tip_mod.get_well_coordinate(self.xM[0],self.yM[0])
            #to_coord.coord_x = to_coord.coord_x + 9.27 + xM[0]*5.07
            #to_coord.coord_y = to_coord.coord_y + 9.07 + yM[0]*5.07
            steps.append(self.move_pos(to_coord, module_dic)) # move over first well
            self.xM[0]=self.xM[0]+1
            to_coord.coord_z = 20.24 #mm
            steps.append(self.move_pos(to_coord, module_dic)) # move over first wellg  git config --global push.default simple

        elif tip_size=="Small":
            tip_mod = module_dic["small_tip_holder"]
            #to_coord = tip_mod.get_mod_coordinates
            if self.xS[0]>7:
                self.yS[0]=self.yS[0]+1
                self.xS[0]=0
            to_coord = tip_mod.get_well_coordinate(self.xS[0],self.yS[0])

            #to_coord.coord_x = to_coord.coord_x + 9.0 + xS[0]*9.0
            #to_coord.coord_y = to_coord.coord_y + 9.37 + yS[0]*9.0
            steps.append(self.move_pos(to_coord, module_dic)) # move over first well
            self.xS[0]=self.xS[0]+1
            to_coord.coord_z = 11.51 #mm
            steps.append(self.move_pos(to_coord, module_dic)) # move over first well

        else:
            print("Error reading tip size")
        return steps

    def eject_tip(self,tip_size, module_dic, steps):

        # TRASH TOP VIEW
        #        __________ (0,0,0) ==> dump_coord (top left corner)
        #      /|_________|       y <-----|
        #     / |         |             / |
        #    /  |         |           v   |
        #    |  |         |         z     V
        #    |  |/\/\/\/\/|               x
        #    |  |---------|
        #    | /         /
        #    |----------|
        #   The dump will always be positioned in this manner
        #       because of the 8-tips pipette.

        to_coord = Coordinate(0,0,0)
        # Get trash_bin from deck
        trash_mod = module_dic["trash"]
        dump_coord  = trash_mod.get_mod_coordinate()
        steps.append(self.move_pos(dump_coord, module_dic))

        # move at the middle of the dump (89.6/2 mm)
        to_coord.coord_y = dump_coord.coord_y+44.8;
        # move over the dump hole
        to_coord.coord_x = dump_coord.coord_x+21;
        steps.append(self.move_pos(to_coord, module_dic)) # x-y move at the same time
        # move just in front of the teeth
        to_coord.coord_x = dump_coord.coord_x+79.1;
        steps.append(self.move_pos(to_coord, module_dic)) # x move (over dump hole)

        # step down
        if tip_size=="Large":
            to_coord.coord_z = 79 #mm
        elif tip_size=="Medium":
            to_coord.coord_z = 51 #mm
        elif tip_size=="Small":
            to_coord.coord_z = 46.5 #mm
        else:
            print("Error reading tip size")
        to_coord.coord_z = dump_coord.coord_z+to_coord.coord_z;
        steps.append(self.move_pos(to_coord, module_dic))

        # move into the teeth (x move)
        if tip_size=="Large":
            to_coord.coord_x = to_coord.coord_x+5.3 #mm
        elif tip_size=="Medium":
            to_coord.coord_x = to_coord.coord_x+2.8 #mm
        elif tip_size=="Small":
            to_coord.coord_x = to_coord.coord_x+1.6 #mm
        else:
            print("Error reading tip size")
        steps.append(self.move_pos(to_coord, module_dic))

        # get up
        if tip_size=="Large":
            to_coord.coord_z = to_coord.coord_z-30.09 #mm
        elif tip_size=="Medium":
            to_coord.coord_z = to_coord.coord_z-20.29 #mm
        elif tip_size=="Small":
            to_coord.coord_z = to_coord.coord_z-11.51 #mm
        else:
            print("Error reading tip size")
        steps.append(self.move_pos(to_coord, module_dic))
        # ready to move away
        return steps

    def parse_mod_coord(self, dest_string, mod_dict):
        dest = dest_string.split("/")
        mod_name = dest[0]
        letter = dest[1][0]
        number = dest[1][1:]
        number = int(number) - 1
        letter = ord(letter) - ord('A')


        if mod_name in mod_dict:
            mod = mod_dict[mod_name]
            return mod.get_well_coordinate(letter, number)

        else:
            print "ERROR no module to set coord"
            self.logger.error("attempt to access the coordinate of a module wich is not reference on the refs section of the json")
            #TODO raise an error
            return None
