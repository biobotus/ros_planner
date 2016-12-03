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
        description = []

        for instruction in json_instruction['groups']:
            if "clap" in instruction:
                val=instruction['clap']
                self._parse_clap(val, module_dic)
                description.append("Clapping {0} times.".format(val))
            else:
                self.logger.error("unknown operation: {0}".format(instruction))
                pass
        return self.steps, description

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

    def gripper_move(self, from_mod, to_mod, height, module_dic, steps):
        from_coord = self.actual_mod_pos(module_dic, from_mod.get_mod_coordinate())
        to_coord = self.actual_mod_pos(module_dic, to_mod.get_mod_coordinate())

        opening = 40

        grab_opening = opening - 5

        cruise_height = height

        steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, cruise_height), module_dic))


        steps.append(self.gripper_pos(-90,opening))

        steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, from_coord.coord_z), module_dic))


        steps.append(self.gripper_pos(-90,grab_opening))


        steps.append(self.move_pos(Coordinate(from_coord.coord_x, from_coord.coord_y, cruise_height), module_dic))

        steps.append(self.move_pos(Coordinate(to_coord.coord_x, to_coord.coord_y, cruise_height), module_dic))

        steps.append(self.move_pos(Coordinate(to_coord.coord_x, to_coord.coord_y, to_coord.coord_z), module_dic))

        steps.append(self.gripper_pos(-90,opening))
        steps.append(self.move_pos(Coordinate(to_coord.coord_x, to_coord.coord_y, cruise_height), module_dic))
        steps.append(self.gripper_pos(90,0))

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
        print('CODOD')
        print(coord)

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

        if mod_name in mod_dict:
            mod = mod_dict[mod_name]
            coord = mod.get_well_coordinate(letter, number)
            return coord

        else:
            print "ERROR no module to set coord"
            self.logger.error("attempt to access the coordinate of a module which is not reference on the refs section of the json")
            #TODO raise an error

            return None
