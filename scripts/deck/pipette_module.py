#!/usr/bin/python

from deck.gripper_tool import GripperTool
from deck.deck_module import Coordinate, DeckModule
from protocol.protocol import Protocol, Step

class PipetteModule(DeckModule):
    def __init__(self, m_type, coord):
        super(PipetteModule, self).__init__(m_type, coord)
        self.logger.info("Pipette initialized")
        self.m_type = m_type
        self.mod_coord = self.get_mod_coordinate()
        self.x_buff = 0
        self.y_buff = 0
        # Constant used for simple pipette across the code
        self.pipette_size = "something"
        self.max_cruise_height_100mm = 100 # z distance used to be more efficient (not returning to z0)
        self.max_column = 7 # pipette box max column (dimension)
        self.large_tip_offset = 52 # large tip offset (zeroing)
        self.medium_tip_offset = 30.37 # medium tip offset (zeroing)
        self.small_tip_offset = 36 # small tip offset (zeroing)
        self.delta_mod_cruise = 20
        if self.m_type == "pipette_m":
            self.max_z_height = 350 # maximum movement in the z-axis
        else:
            self.max_z_height = 350

        self.dump_coord_y_offset = 10.5 # necessary offset to move in the middle of the teeth of the trash (ALL PIPETTE)
        self.dump_coord_x_offset = 79.1 # necessary offset to move in front of the teeth of the trash (ALL PIPETTE)
        self.large_tip_penetration_depth_s = 0
        self.medium_tip_penetration_depth_s = 6
        self.small_tip_penetration_depth_s = 4
        self.large_tip_penetration_depth_m = 1
        self.medium_tip_penetration_depth_m = 6
        self.small_tip_penetration_depth_m = 4

        self.large_tip_length = 81
        self.medium_tip_length = 53
        self.small_tip_length = 49
        self.large_tip_trash_x_offset = 20  # x distance to enter the teeth's trash properly
        self.medium_tip_trash_x_offset = 29  # x distance to enter the teeth's trash properly
        self.small_tip_trash_x_offset = 29  # x distance to enter the teeth's trash properly
        self.max_volume = 800 # maximum quantity in uL of volume that can be pipetted by the large tip
        self.med_volume = 100 # quantity in uL of volume that can be pipetted by the medium tip
        self.min_volume = 10 # minimum quantity in uL of volume that can be pipetted by the small tip

    def parse_json(self, json_instruction, module_dic):
        self.steps = []
        description = ["Use {0} to execute the following actions:".format(self.m_type)]

        trash_mod = module_dic["trash"]
        dump_coord  = self.actual_mod_pos(module_dic, trash_mod.get_mod_coordinate())
        self.cruise_height = dump_coord.coord_z-self.max_cruise_height_100mm
        self.mod_cruise_height = self.cruise_height

        for instruction in json_instruction['groups']:
            if "distribute" in instruction:
                #TODO raise an exception, this operation won't be possible with the mechaPipette
                self.logger.error("distribute operation is not possible with the mecha pipette for now")

            elif "consolidate" in instruction:
                #TODO raise an exception, this operation won't be possible with the mechaPipette
                self.logger.error("consolidate operation is not possible with the mecha pipette for now")

            elif "transfer" in instruction:
                val = instruction['transfer']
                self._parse_transfer(val, module_dic)
                description.append("Transfer {0} &micro;L from {1} to {2}.".format(val['volume'], \
                                                                          val['from'], val['to']))

            elif "multi_dispense" in instruction:
                #TODO est possible avec la mechaPipette?
                val = instruction['multi_dispense']
                self._parse_multi_dispense(val, module_dic)
                description.append("Multi dispense {0} &micro;L {1} times from {2} to {3}.".format(val['volume'], \
                                                                       val['iteration'], val['from'], val['to']))

            elif "serial_dilution" in instruction:
                #TODO impossible with pipette_s
                val = instruction['serial_dilution']
                self._parse_serial_dilution(val, module_dic)
                description.append("Serial dilution of {0} &micro;L {1} times with {2} iterations from {3} to {4}.".format(val['volume'], \
                                                                         val['iteration'], val['mix_iteration'], val['from'], val['to']))

            elif "mix" in instruction:
                #TODO impossible with mechaPipette?
                val = instruction['mix']
                self._parse_mix(val, module_dic)
                description.append("Mix {0} with {1} aspiration/dispense cycles of {2} &micro;L.".format(val['from'], \
                                                                                    val['iteration'], val['volume']))

            else:
                #TODO raise an exception instruction not known
                self.logger.error("unknown operation: {0}".format(instruction))
                pass

        return self.steps, description

    def _parse_transfer(self, trans_json, module_dic):
        """
        A transfer is an operation which transfer from one well to another a
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
        self.logger.info("parsing transfer instruction")
        print("parsing transfer instruction")

        # Coord initialized
        from_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(trans_json["from"], module_dic))
        to_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(trans_json["to"], module_dic))

        trash_mod = module_dic["trash"]
        dump_coord  = self.actual_mod_pos(module_dic, trash_mod.get_mod_coordinate())

        # Get tip from tip holder module
        self.get_tip_size(trans_json["volume"])
        self.get_tip(self.pipette_size, module_dic)

        from_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(trans_json["from"], module_dic))
        to_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(trans_json["to"], module_dic))

        print('from coord')
        print(from_coord)

        from_mod = trans_json["from"].split("/")
        from_mod = from_mod[0]

        to_mod = trans_json["to"].split("/")
        to_mod = to_mod[0]

        # Going to the source position
        self.steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, self.cruise_height), module_dic))

        # Getting down
        self.steps.append(self.move_pos(from_coord, module_dic))

        # aspirate
        self.steps.append(self.aspirate(trans_json["volume"],trans_json["aspirate_speed"]))
        # get Up
        if from_mod == to_mod:
            print('SAME MODULE')
            self.mod_cruise_height = from_coord.coord_z - self.delta_mod_cruise
            self.steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, self.mod_cruise_height), module_dic))
        else:
            print('NOT SAME MODULE')
            self.steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, self.cruise_height), module_dic))

        # got to the destination well
        if from_mod == to_mod:
            print('SAME MODULE')
            self.mod_cruise_height = to_coord.coord_z - self.delta_mod_cruise
            self.steps.append(self.move_pos(Coordinate(to_coord.coord_x, to_coord.coord_y, self.mod_cruise_height), module_dic))
        else:
            self.steps.append(self.move_pos(Coordinate(to_coord.coord_x, to_coord.coord_y, self.cruise_height), module_dic))

        # getting down
        self.steps.append(self.move_pos(to_coord, module_dic))
        # blow
        self.steps.append(self.dispense(trans_json["volume"], trans_json["dispense_speed"]))
        # get up
        self.cruise_height = dump_coord.coord_z-self.max_cruise_height_100mm
        self.steps.append(self.move_pos(Coordinate(to_coord.coord_x, to_coord.coord_y, self.cruise_height), module_dic))
        self.get_tip_size(trans_json["volume"])
        self.eject_tip(self.pipette_size, module_dic)

        # TODO: REMOVE
        #gripper_mod = module_dic["gripper"]
        #gripper_mod.gripper_move(trans_json["from"],trans_json["to"],self.cruise_height,module_dic,self.steps)
        #gripper_mod.gripper_clap(10,module_dic,self.steps)

        return

    def _parse_multi_dispense(self, trans_json, module_dic):
        """
        A multi dispense is an operation which go to a well divide the volume into other wells
        equally.
        There is 8 step to do so :
            * get the tip
            * go to the source well position
            * get down
            * aspirate
            * get up                        \
            * go to the destination well     \
            * get down                       /  loop
            * dispense                      /
            * get up
            * go to the dump
            * eject tip
        :param trans_json: the json containing the information
        :param module_dict: the list of module involved in the protocol
        :return: a list of steps with their different parameters to complete
            the task
        """
        self.logger.info("parsing multi dispense instruction")
        same_mod = 0
        # Coord initialized
        from_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(trans_json["from"], module_dic))
        to_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(trans_json["to"], module_dic))

        trash_mod = module_dic["trash"]
        dump_coord  = self.actual_mod_pos(module_dic, trash_mod.get_mod_coordinate())
        # go to max cruise_height
        self.cruise_height = dump_coord.coord_z - self.max_cruise_height_100mm

        # Get tip from tip holder module
        self.get_tip_size(trans_json["volume"])
        self.get_tip(self.pipette_size, module_dic)

        from_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(trans_json["from"], module_dic))
        to_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(trans_json["to"], module_dic))

        from_mod = trans_json["from"].split("/")
        from_mod = from_mod[0]

        to_mod = trans_json["to"].split("/")
        to_mod = to_mod[0]


        # Going to the source position
        if from_mod == to_mod:
            self.mod_cruise_height = from_coord.coord_z - self.delta_mod_cruise
            self.steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, self.mod_cruise_height), module_dic))
        else:
            self.steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, self.cruise_height), module_dic))

        # Getting down
        self.steps.append(self.move_pos(from_coord, module_dic))

        # aspirate
        self.steps.append(self.aspirate(trans_json["volume"],
                                   trans_json["aspirate_speed"]))

        if from_mod == to_mod:
            self.mod_cruise_height = from_coord.coord_z - self.delta_mod_cruise
            self.steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, self.mod_cruise_height), module_dic))
        else:
            self.steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, self.cruise_height), module_dic))

        iteration = int(trans_json["iteration"])
        k = 0
        last = 0
        for k in range(iteration):
            if k == iteration-1:
                last = 1
            # move from one well to another and blow
            self.multi_dispense(last, iteration, module_dic, float(trans_json["volume"]), float(trans_json["dispense_speed"]), trans_json)
        # get up
        # Go to 1st well from deck
        module =  trans_json["to"].split("/")
        tip_mod = module_dic[str(module[0])]
        letter = module[1][0]
        number = module[1][1:]
        number = int(number) - 1 + self.x_buff
        letter = ord(letter) - ord('A') + self.y_buff

        to_coord = self.actual_mod_pos(module_dic, tip_mod.get_well_coordinate(letter, number))

        self.cruise_height = dump_coord.coord_z-self.max_cruise_height_100mm
        self.steps.append(self.move_pos(Coordinate(to_coord.coord_x, to_coord.coord_y, self.cruise_height), module_dic))
        self.get_tip_size(trans_json["volume"])
        self.eject_tip(self.pipette_size, module_dic)
        self.x_buff = 0
        self.y_buff = 0

        return

    def _parse_mix(self, trans_json, module_dic):
        """
        A mix is an operation which go to a well and mix the content of it
        by aspiring and ejecting a percentage of the liquid inside the specified well
        There is 8 step to do so :
            * get the tip
            * go to the well position
            * get down
            * aspirate      \
                              >  loop
            * dispense      /
            * get up
            * go to the dump
            * eject tip
        :param trans_json: the json containing the information
        :param module_dict: the list of module involved in the protocol
        :return: a list of steps with their different parameters to complete
            the task
        """
        self.logger.info("parsing mix instruction")
        # Coord initialized
        from_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(trans_json["from"], module_dic))
        #to_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(trans_json["to"], module_dic))

        trash_mod = module_dic["trash"]
        dump_coord  = self.actual_mod_pos(module_dic, trash_mod.get_mod_coordinate())
        # go to max cruise_height
        self.cruise_height = dump_coord.coord_z-self.max_cruise_height_100mm

        # Get tip from tip holder module
        self.get_tip_size(trans_json["volume"])
        self.get_tip(self.pipette_size, module_dic)

        from_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(trans_json["from"], module_dic))

        # Going to the source position
        if from_mod == to_mod:
            self.mod_cruise_height = from_coord.coord_z - self.delta_mod_cruise
            self.steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, self.mod_cruise_height), module_dic))
        else:
            self.steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, self.cruise_height), module_dic))

        # Getting down
        self.steps.append(self.move_pos(from_coord, module_dic))
        # aspirate 75% of the volume in the well
        self.steps.append(self.aspirate(float(trans_json["volume"])*0.75,
                                           trans_json["aspirate_speed"]))
        k=0
        number_of_iteration = int(trans_json["iteration"])
        for k in range(number_of_iteration):

            # blow 90% of the volume in the tip (90% of the 75% from the total volume)
            self.steps.append(self.dispense(float(trans_json["volume"])*0.75*0.9,
                                    trans_json["dispense_speed"]))
            # aspirate
            self.steps.append(self.aspirate(float(trans_json["volume"])*0.75*0.9,
                                   trans_json["aspirate_speed"]))

        # Once the mix is done, blow all the liquid out
        self.steps.append(self.dispense(float(trans_json["volume"])*0.75,
                                    trans_json["dispense_speed"]))
        if from_mod == to_mod:
            self.mod_cruise_height = from_coord.coord_z - self.delta_mod_cruise
            self.steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, self.mod_cruise_height), module_dic))
        else:
            self.steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, self.cruise_height), module_dic))

        self.get_tip_size(trans_json["volume"])
        self.eject_tip(self.pipette_size, module_dic)

        return

    def _parse_serial_dilution(self, trans_json, module_dic):
        """
        A transfer is an operation which transfer from one well to another a
        certain amount of liquid
        There is 16 and more steps to do so :
            * get the tip
            * go to the source well position
            * get down
            * aspirate
            * get up

            * go to the destination well  \
            * get down                     \
            * dispense (90%)                > loop
            * aspirate (75%)               /
            * get up                      /

            * go to the destination well
            * get down
            * dispense (100%)
            * get up
            * go to the dump
            * eject tip
        :param trans_json: the json containing the information
        :param module_dict: the list of module involved in the protocol
        :return: a list of steps with their different parameters to complete
            the task
        """
        self.logger.info("parsing serial dilution instruction")

        # Coord initialized
        from_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(trans_json["from"], module_dic))
        to_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(trans_json["to"], module_dic))

        trash_mod = module_dic["trash"]
        dump_coord  = self.actual_mod_pos(module_dic, trash_mod.get_mod_coordinate())
        # go to max cruise_height
        self.cruise_height = dump_coord.coord_z-self.max_cruise_height_100mm

        # Get tip from tip holder module
        self.get_tip_size(trans_json["volume"])
        self.get_tip(self.pipette_size, module_dic)

        # go to the source well position
        from_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(trans_json["from"], module_dic))
        to_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(trans_json["to"], module_dic))

        from_mod = trans_json["from"].split("/")
        from_mod = from_mod[0]

        to_mod = trans_json["to"].split("/")
        to_mod = to_mod[0]

        # Going to the source position
        if from_mod == to_mod:
            self.mod_cruise_height = from_coord.coord_z - self.delta_mod_cruise
            self.steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, self.mod_cruise_height), module_dic))
        else:
            self.steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, self.cruise_height), module_dic))

        # Getting down
        self.steps.append(self.move_pos(from_coord, module_dic))

        # aspirate
        self.steps.append(self.aspirate(trans_json["volume"],
                                   trans_json["aspirate_speed"]))

        # get Up
        if from_mod == to_mod:
            self.mod_cruise_height = from_coord.coord_z - self.delta_mod_cruise
            self.steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, self.mod_cruise_height), module_dic))
        else:
            self.steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, self.cruise_height), module_dic))

        last_mix = 0
        k=0
        number_of_iteration = int(trans_json["iteration"])
        # go to the module specified and blow 75% re-aspirate 90% of volume, go to next well n times
        for k in range(number_of_iteration):
            # blow + aspirate with option turn on
            if k == number_of_iteration-1:
                last_mix = 1
            self.serial_dilution(number_of_iteration, module_dic, float(trans_json["volume"]), trans_json["aspirate_speed"], trans_json, last_mix)

        # get up
        # Go to 1st well from deck
        module =  trans_json["to"].split("/")
        tip_mod = module_dic[str(module[0])]
        letter = module[1][0]
        number = module[1][1:]
        number = int(number) - 1 + self.x_buff
        letter = ord(letter) - ord('A') + self.y_buff

        to_coord = self.actual_mod_pos(module_dic, tip_mod.get_well_coordinate(letter, number))

        self.steps.append(self.move_pos(Coordinate(to_coord.coord_x, to_coord.coord_y, self.cruise_height), module_dic))
        self.get_tip_size(trans_json["volume"])
        self.eject_tip(self.pipette_size, module_dic)
        self.x_buff = 0
        self.y_buff = 0

        return

    def aspirate(self, volume, speed):

        args = {"vol": float(volume), "speed": float(speed)}
        params = {"name": "manip", "args": args}
        step_move = Step({"module_type": self.m_type, "params": params})

        return step_move

    def dispense(self, volume, speed):

        args = {"vol": -float(volume), "speed": float(speed)}
        params = {"name": "manip", "args": args}
        step_move = Step({"module_type": self.m_type, "params": params})

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
        elif int(coord.coord_x)>1050 or int(coord.coord_y)>800:
            print("Over coord x: {} y: {} z: {}"\
                .format(coord.coord_x, coord.coord_y, coord.coord_z))
            self.logger.error("Received a too far coord x: {} y: {} z: {}"\
                .format(coord.coord_x, coord.coord_y, coord.coord_z))
            return -1

        else:
            args =  {"x": coord.coord_x, "y": coord.coord_y, "z": coord.coord_z}
            params = {"name": "pos", "args": args}
            step_move = Step({"module_type": self.m_type, "params": params})

        return step_move

    def get_tip(self, tip_size, module_dic):

        to_coord = Coordinate(0,0,0)
            # Get tip holder from deck
        if tip_size=="Large":
            tip_mod = module_dic["large_tip_holder"]
            # go over the 1st well
            tip_pos = tip_mod.get_tip_pos(self.m_type)
            to_coord = self.actual_mod_pos(module_dic, tip_mod.get_well_coordinate(tip_pos[0], tip_pos[1]))
            to_coord.coord_z = self.cruise_height
            self.steps.append(self.move_pos(to_coord, module_dic))
            # get in the tip
            to_coord = self.actual_mod_pos(module_dic, tip_mod.get_well_coordinate(tip_pos[0], tip_pos[1]))

            if self.m_type == "pipette_m":
                to_coord.coord_z = to_coord.coord_z+self.large_tip_penetration_depth_m
            else:
                to_coord.coord_z = to_coord.coord_z+self.large_tip_penetration_depth_s

            self.steps.append(self.move_pos(to_coord, module_dic))
            to_coord.coord_z = self.cruise_height
            self.steps.append(self.move_pos(to_coord, module_dic))
            # move up
            self.mod_coord.coord_z = self.mod_coord.coord_z + self.large_tip_offset
            self.tip_height = self.max_z_height - self.large_tip_offset


        elif tip_size=="Medium":

            tip_mod = module_dic["medium_tip_holder"]
            # go over the 1st well
            tip_pos = tip_mod.get_tip_pos(self.m_type)
            to_coord = self.actual_mod_pos(module_dic, tip_mod.get_well_coordinate(tip_pos[0], tip_pos[1], ))
            to_coord.coord_z = self.cruise_height
            self.steps.append(self.move_pos(to_coord, module_dic))
            # get in the tip
            to_coord = self.actual_mod_pos(module_dic, tip_mod.get_well_coordinate(tip_pos[0], tip_pos[1], ))

            if self.m_type == "pipette_m":
                to_coord.coord_z = to_coord.coord_z+self.medium_tip_penetration_depth_m
            else:
                to_coord.coord_z = to_coord.coord_z+self.medium_tip_penetration_depth_s

            self.steps.append(self.move_pos(to_coord, module_dic))
            to_coord.coord_z = self.cruise_height
            self.steps.append(self.move_pos(to_coord, module_dic))
            # move up
            self.mod_coord.coord_z = self.mod_coord.coord_z + self.medium_tip_offset #tip offset
            self.tip_height = self.max_z_height - self.medium_tip_offset


        elif tip_size=="Small":
            tip_mod = module_dic["small_tip_holder"]
            # go over the 1st well
            tip_pos = tip_mod.get_tip_pos(self.m_type)
            to_coord = self.actual_mod_pos(module_dic, tip_mod.get_well_coordinate(tip_pos[0],tip_pos[1], ))
            to_coord.coord_z = self.cruise_height
            self.steps.append(self.move_pos(to_coord, module_dic))
            # get in the tip
            to_coord = self.actual_mod_pos(module_dic, tip_mod.get_well_coordinate(tip_pos[0],tip_pos[1], ))
            if self.m_type == "pipette_m":
                to_coord.coord_z = to_coord.coord_z+self.small_tip_penetration_depth_m
            else:
                to_coord.coord_z = to_coord.coord_z+self.small_tip_penetration_depth_s

            self.steps.append(self.move_pos(to_coord, module_dic))
            to_coord.coord_z = self.cruise_height
            # move up
            self.steps.append(self.move_pos(to_coord, module_dic))
            self.mod_coord.coord_z = self.mod_coord.coord_z + self.small_tip_offset # tip offset
            self.tip_height = self.max_z_height - self.small_tip_offset

        else:
            print("Error reading tip size")

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
        # move in the middle of the trash
        to_coord.coord_y = dump_coord.coord_y+self.dump_coord_y_offset
        to_coord.coord_x = dump_coord.coord_x+self.dump_coord_x_offset
        # self.steps.append(self.move_pos(to_coord, module_dic)) # x-y move (over dump hole)
        # move down
        if tip_size=="Large":
            to_coord.coord_z = self.large_tip_length #mm
        elif tip_size=="Medium":
            to_coord.coord_z = self.medium_tip_length #mm
        elif tip_size=="Small":
            to_coord.coord_z = self.small_tip_length #mm
        else:
            print("Error reading tip size")
        to_coord.coord_z = dump_coord.coord_z+to_coord.coord_z
        self.steps.append(self.move_pos(to_coord, module_dic))
        # move into the teeth to take out the tip
        if tip_size=="Large":
            to_coord.coord_x = to_coord.coord_x + self.large_tip_trash_x_offset
        elif tip_size=="Medium":
            to_coord.coord_x = to_coord.coord_x + self.medium_tip_trash_x_offset
        elif tip_size=="Small":
            to_coord.coord_x = to_coord.coord_x + self.small_tip_trash_x_offset
        else:
            print("Error reading tip size")
        self.steps.append(self.move_pos(to_coord, module_dic))

        # ready to move away
        to_coord.coord_z = self.cruise_height
        self.steps.append(self.move_pos(to_coord, module_dic))

        #Remove tip offset
        if tip_size=="Large":
            self.mod_coord.coord_z = self.mod_coord.coord_z - self.large_tip_offset
        elif tip_size=="Medium":
            self.mod_coord.coord_z = self.mod_coord.coord_z - self.medium_tip_offset
        elif tip_size=="Small":
            self.mod_coord.coord_z = self.mod_coord.coord_z - self.small_tip_offset
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
            self.logger.error("attempt to access the coordinate of a module which is not reference on the refs section of the json")
            #TODO raise an error
            return None

    def get_tip_size(self,volume):
        # Volume is in uL
        volume = float(volume)
        if volume  > self.med_volume and volume <= self.max_volume:
            self.pipette_size = "Large"

        elif volume > self.min_volume and volume <= self.med_volume:
            self.pipette_size = "Medium"

        elif volume <= self.min_volume:
            self.pipette_size = "Small"

        else:
            print("Unable to determine the correct tip size to use")
            self.logger.error("attempt to calculate best tip size to use, but unable to. Volume seems out of range!")

        return

    def multi_dispense(self, last, iteration, module_dic, volume, speed, trans_json):
        # Go to 1st well from deck
        module =  trans_json["to"].split("/")
        tip_mod = module_dic[str(module[0])]
        letter = module[1][0]
        number = module[1][1:]
        number = int(number) - 1 + self.x_buff
        letter = ord(letter) - ord('A') + self.y_buff


        # go over the 1st well
        to_coord = self.actual_mod_pos(module_dic, tip_mod.get_well_coordinate(letter, number))

        self.mod_cruise_height = to_coord.coord_z - self.delta_mod_cruise
        self.steps.append(self.move_pos(Coordinate(to_coord.coord_x, to_coord.coord_y, self.mod_cruise_height), module_dic))

        #Get down
        self.steps.append(self.move_pos(to_coord, module_dic))

        # blow
        self.steps.append(self.dispense(volume/iteration, speed))
        # move up
        self.steps.append(self.move_pos(Coordinate(to_coord.coord_x, to_coord.coord_y, self.mod_cruise_height), module_dic))

        self.y_buff=self.y_buff+1

        # if the well has been blown in, go to an other well
        if self.m_type == "pipette_m":
            self.x_buff=self.x_buff+1
            self.y_buff = 0
        else:
            if self.y_buff>self.max_column:
                self.x_buff=self.x_buff+1
                self.y_buff=0

        if last ==1:
            self.y_buff=self.y_buff-1

        if self.m_type == "pipette_m":
            if last == 1:
                self.x_buff=self.x_buff-1
                self.y_buff=0
        return

    def serial_dilution(self, iteration, module_dic, volume, speed, trans_json, last_mix):
        to_coord = Coordinate(0,0,0)

        # Go to 1st well from deck
        module =  trans_json["to"].split("/")
        tip_mod = module_dic[str(module[0])]
        letter = module[1][0]
        number = module[1][1:]
        number = int(number) - 1 + self.x_buff
        letter = ord(letter) - ord('A') + self.y_buff

        # go over the 1st well
        to_coord = self.actual_mod_pos(module_dic, tip_mod.get_well_coordinate(letter, number))
        self.mod_cruise_height = to_coord.coord_z - self.delta_mod_cruise
        self.steps.append(self.move_pos(Coordinate(to_coord.coord_x, to_coord.coord_y, self.mod_cruise_height), module_dic))

        #Get down
        self.steps.append(self.move_pos(to_coord, module_dic))
        k=0
        mix_iteration = int(trans_json["mix_iteration"])
        for k in range(mix_iteration):
            # blow
            self.steps.append(self.dispense(volume, speed))
            # aspirate
            self.steps.append(self.aspirate(volume, speed))

        if last_mix == 1:
            print "DISPENSE DONE"
            self.steps.append(self.dispense(volume, speed))
        # move up
        self.steps.append(self.move_pos(Coordinate(to_coord.coord_x, to_coord.coord_y, self.mod_cruise_height), module_dic))
        self.y_buff=self.y_buff+1
        # if the well has been blown in, go to an other well
        if self.m_type == "pipette_m":
            self.x_buff=self.x_buff+1
            self.y_buff = 0
        else:
            if self.y_buff>self.max_column:
                self.x_buff=self.x_buff+1
                self.y_buff=0

        if last_mix ==1:
            self.y_buff=self.y_buff-1

        if self.m_type == "pipette_m":
            if last_mix == 1:
                self.x_buff=self.x_buff-1
                self.y_buff=0

        return

