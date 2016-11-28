#!/usr/bin/python

from deck.deck_module import Coordinate, DeckModule
from protocol.protocol import Protocol, Step

class GripperTool(DeckModule):
    def __init__(self, m_type, coord):
        super(GripperTool, self).__init__(m_type, coord)
        self.logger.info("Gripper initialized")
        self.m_type = m_type
        self.mod_coord = self.get_mod_coordinate()
        self.x_buff = 0
        self.y_buff = 0
        # Constant used for simple pipette across the code
        self.wrist_pos_max = 90
        self.wrist_pos_min = -90
        self.opening_pos_max = 100
        self.opening_pos_min = 0

    def parse_json(self, json_instruction, module_dic):
        self.steps = []

        for instruction in json_instruction['groups']:
            if "clap" in instruction:
                self._parse_clap(instruction['clap'], module_dic)
                description.append("Clapping {0} times.".format(val['clap']))
            else:
                self.logger.error("unknown operation: {0}".format(instruction))
                pass
        return self.steps

    def _parse_clap(self,clap, module_dic):
        self.steps.append(self.gripper_pos(-90,0))
        self.steps.append(self.gripper_pos(45,0))
        for i in range(clap):
                self.steps.append(self.gripper_pos(45,100))
                self.steps.append(self.gripper_pos(45,0))
        self.steps.append(self.gripper_pos(90,0))
        return

    def gripper_clap(self,clap, module_dic, steps):
        steps.append(self.gripper_pos(-90,0))
        steps.append(self.gripper_pos(45,0))
        for i in range(clap):
                steps.append(self.gripper_pos(45,25))
                steps.append(self.gripper_pos(45,0))
        steps.append(self.gripper_pos(90,0))
        return

    def gripper_move(self, from_mod, to_mod,height, module_dic, steps):
        from_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(from_mod, module_dic))
        to_coord = self.actual_mod_pos(module_dic, self.parse_mod_coord(to_mod, module_dic))

        steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, height), module_dic))

        #TODO Add module diameter for opening

        steps.append(self.gripper_pos(-90,0))
        steps.append(self.gripper_pos(-90,100))
        steps.append(self.gripper_pos(-90,0))

        steps.append(self.move_pos(Coordinate(to_coord.coord_x, to_coord.coord_y, height), module_dic))

        steps.append(self.gripper_pos(-90,100))

        return

    def gripper_pos(self, wrist, opening):

        if self.wrist_pos_max>=int(wrist)>=self.wrist_pos_min and self.opening_pos_max>=int(opening)>=self.opening_pos_min :
            args = {"wrist": float(wrist), "opening": float(opening)} #, "speed": float(speed)}
            params = {"name": "manip", "args": args}
            step_move = Step({"module_type": self.m_type, "params": params})
            return step_move
        else:
            return -1

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

    def actual_mod_pos(self, module_dic, coord):
        coord.coord_x = coord.coord_x-self.mod_coord.coord_x
        coord.coord_y = coord.coord_y-self.mod_coord.coord_y
        coord.coord_z = coord.coord_z-self.mod_coord.coord_z
        return coord

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

