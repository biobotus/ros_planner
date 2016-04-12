
#/usr/bin/

from deck.deck_module import Coordinate, DeckModule, ModuleParam
from protocol.protocol import Protocol, Step, StepParameter

class PipetteModule(DeckModule):

    def __init__(self, name, coord, m_type):
        super(PipetteModule, self).__init__(name, coord)
        #self.add_parameter(ModuleParam("volume", True, False))
        #self.add_parameter(ModuleParam("from", True, False))
        #self.add_parameter(ModuleParam("to", True, False))
        self.logger.info("Pipette initialized")
        self.m_type = m_type
        self.mod_coord = self.get_mod_coordinate()
        self.xL=[0]
        self.yL=[0]
        self.xM=[0]
        self.yM=[0]
        self.xS=[0]
        self.yS=[0]

    def parse_json(self, json_instruction, module_dic):
        self.steps = []

        for instruction in json_instruction:
            if "distribute" in instruction:
                #TODO raise an exception, this operation won't be possible with the mechaPipette
                self.logger.error("distribute operation is not possible with the mecha pipette for now")

            elif "consolidate" in instruction:
                #TODO raise an exception, this operation won't be possible with the mechaPipette
                self.logger.error("consolidate operation is not possible with the mecha pipette for now")

            elif "transfert" in instruction:
                self._parse_transfert(instruction['transfert'], module_dic)

            elif "mix" in instruction:
                #TODO est possible avec la mechaPipette?
                self._parse_mix(instruction['mix'])
            else:
                #TODO raise an exception instruction not known
                self.logger.error("unknown operation: {0}".format(instruction))
                pass
        return self.steps


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
        print "START TRANSFER"
        self.logger.info("parsing transfert instruction")



        # Coord initialized
        from_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(trans_json["from"], module_dic))
        to_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(trans_json["to"], module_dic))
        
        trash_mod = module_dic["trash"]
        dump_coord  = self.actual_mod_pos(module_dic, trash_mod.get_mod_coordinate())
        self.height = dump_coord.coord_z-100

        
        # Get tip from tip holder module
        self.get_tip("Small", module_dic)

        from_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(trans_json["from"], module_dic))
        to_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(trans_json["to"], module_dic))

        from_mod = trans_json["from"].split("/")
        from_mod = from_mod[0]
        
        to_mod = trans_json["to"].split("/")
        to_mod = to_mod[0]

        # Going to the source position
        print("Going to the source position")
        self.steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, self.height), module_dic))

        # Getting down
        print("Getting down")
        self.steps.append(self.move_pos(from_coord, module_dic))

        # aspirate
        #self.steps.append(self.aspirate(trans_json["volume"],
        #                           trans_json["aspirate_speed"]))

        if from_mod == to_mod:
            print "CHANGIGN HEIGHT"
            self.height = self.tip_height

        # get Up
        print("get Up")
        self.steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, self.height), module_dic))

        # got to the destination well
        
        print("got to the destination well")
        self.steps.append(self.move_pos(Coordinate(to_coord.coord_x, to_coord.coord_y, self.height), module_dic))
        # getting down
        print("getting down")
        self.steps.append(self.move_pos(to_coord, module_dic))
        # blow
        print("blow")
        #self.steps.append(self.dispense(trans_json["volume"],
        #                            trans_json["dispense_speed"]))
        # get up
        self.height = dump_coord.coord_z-100
        print("get up")
        self.steps.append(self.move_pos(Coordinate(to_coord.coord_x, to_coord.coord_y, self.height), module_dic))
        print("eject that")
        self.eject_tip("Small", module_dic)
        return 

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

    def actual_mod_pos(self, module_dic, coord):

        coord.coord_x = coord.coord_x-self.mod_coord.coord_x
        coord.coord_y = coord.coord_y-self.mod_coord.coord_y
        coord.coord_z = coord.coord_z-self.mod_coord.coord_z
        return coord

    def move_pos(self, coord, module_dic):

        if int(coord.coord_x)<0 or int(coord.coord_y)<0 or int(coord.coord_z) < 0:
            print("Negative coord x: {} y: {} z: {}"\
                .format(coord.coord_x, coord.coord_y, coord.coord_z))
            self.logger.error("Received a negative coordinate x: {} y: {} z: {}"\
                .format(coord.coord_x, coord.coord_y, coord.coord_z))
            return -1
        else:
            args =  {"x": coord.coord_x, "y": coord.coord_y, "z": coord.coord_z}
            params = {"name": "pos", "args": args}
            step_move = Step({"module_type": self.m_type, "params": params})

            print(step_move)
        return step_move

    def get_tip(self, tip_size, module_dic):

        to_coord = Coordinate(0,0,0)
            # Get tip holder from deck
        if tip_size=="Large":
            tip_mod = module_dic["large_tip_holder"]
            to_coord = tip_mod.get_mod_coordinate()
            if self.xL[0]>7:
                self.yL[0]=self.yL[0]+1
                self.xL[0]=0
            to_coord = self.actual_mod_pos(module_dic, tip_mod.get_well_coordinate(self.xL[0],self.yL[0], ))
            to_coord.coord_z = self.height
            self.steps.append(self.move_pos(to_coord, module_dic))
            to_coord = self.actual_mod_pos(module_dic, tip_mod.get_well_coordinate(self.xL[0],self.yL[0], ))
            self.xL[0]=self.xL[0]+1
            to_coord.coord_z = to_coord.coord_z-30.09 #mm
            print(to_coord)
            self.steps.append(self.move_pos(to_coord, module_dic)) # move over first well

        elif tip_size=="Medium":
            tip_mod = module_dic["medium_tip_holder"]
            #to_coord = tip_mod.get_mod_coordinate()
            if self.xM[0]>7:
                self.yM[0]=self.yM[0]+1
                self.xM[0]=0
            
            to_coord = self.actual_mod_pos(module_dic, tip_mod.get_well_coordinate(self.xM[0],self.yM[0], ))
            to_coord.coord_z = self.height
            self.steps.append(self.move_pos(to_coord, module_dic))
            to_coord = self.actual_mod_pos(module_dic, tip_mod.get_well_coordinate(self.xM[0],self.yM[0], ))
            self.xM[0]=self.xM[0]+1
            to_coord.coord_z = to_coord.coord_z-20.24 #mm
            self.steps.append(self.move_pos(to_coord, module_dic)) # move over first wellg  git config --global push.default simple

        elif tip_size=="Small":
            tip_mod = module_dic["small_tip_holder"]
            #to_coord = tip_mod.get_mod_coordinates
            if self.xS[0]>7:
                self.yS[0]=self.yS[0]+1
                self.xS[0]=0

            to_coord = self.actual_mod_pos(module_dic, tip_mod.get_well_coordinate(self.xS[0],self.yS[0], ))
            to_coord.coord_z = self.height
            self.steps.append(self.move_pos(to_coord, module_dic))

            to_coord = self.actual_mod_pos(module_dic, tip_mod.get_well_coordinate(self.xS[0],self.yS[0], ))
            to_coord.coord_z = to_coord.coord_z+6.51 #11.51mm
            self.steps.append(self.move_pos(to_coord, module_dic)) # move over first well
            to_coord.coord_z = self.height
            self.steps.append(self.move_pos(to_coord, module_dic))

            self.xS[0]=self.xS[0]+1
            self.mod_coord.coord_z = self.mod_coord.coord_z + 37 #tip offset
            self.tip_height = 330 - 37
        else:
            print("Error reading tip size")
        print self.steps
        return 

    def eject_tip(self,tip_size, module_dic):

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
        dump_coord  = self.actual_mod_pos(module_dic, trash_mod.get_mod_coordinate())
        to_coord.coord_z = dump_coord.coord_z

#        self.steps.append(self.move_pos(dump_coord, module_dic))
        print(' move at the middle of the dump (89.6/2 mm)')
        to_coord.coord_y = dump_coord.coord_y+50
        print(' move over the dump hole')
#        to_coord.coord_x = dump_coord.coord_x+21
#        self.steps.append(self.move_pos(to_coord, module_dic)) # x move
        print(' move just in front of the teeth')
        to_coord.coord_x = dump_coord.coord_x+79.1
        self.steps.append(self.move_pos(to_coord, module_dic)) # x move (over dump hole)
        # Both x movement are now separate (last ones) but they could eventually be 
        # added together for simplicity and speed

        print(' step down')
        if tip_size=="Large":
            to_coord.coord_z = 79 #mm
        elif tip_size=="Medium":
            to_coord.coord_z = 51 #mm
        elif tip_size=="Small":
            to_coord.coord_z = 48.2 #mm
        else:
            print("Error reading tip size")
        to_coord.coord_z = dump_coord.coord_z+to_coord.coord_z
        self.steps.append(self.move_pos(to_coord, module_dic))

        print(" move into the teeth (x move)")
        if tip_size=="Large":
            to_coord.coord_x = to_coord.coord_x+5.3 #mm
        elif tip_size=="Medium":
            to_coord.coord_x = to_coord.coord_x+2.8 #mm
        elif tip_size=="Small":
            to_coord.coord_x = to_coord.coord_x+28.17 #mm
        else:
            print("Error reading tip size")
        self.steps.append(self.move_pos(to_coord, module_dic))

        # ready to move away
        to_coord.coord_z = self.height
        self.steps.append(self.move_pos(to_coord, module_dic))

        if tip_size=="Large":
            self.mod_coord.coord_z = self.mod_coord.coord_z - 37
        elif tip_size=="Medium":
            self.mod_coord.coord_z = self.mod_coord.coord_z - 37
        elif tip_size=="Small":
            self.mod_coord.coord_z = self.mod_coord.coord_z - 37
        else:
            print("Error reading tip size")

        return

    def parse_mod_coord(self, dest_string, mod_dict):
        dest = dest_string.split("/")
        mod_name = dest[0]
        letter = dest[1][0]
        number = dest[1][1:]
        number = int(number) - 1
        letter = ord(letter) - ord('A')
        

        if mod_name in mod_dict:
            mod = mod_dict[mod_name]
            coord = mod.get_well_coordinate(letter, number)
            return coord

        else:
            print "ERROR no module to set coord"
            self.logger.error("attempt to access the coordinate of a module wich is not reference on the refs section of the json")
            #TODO raise an error
            return None
