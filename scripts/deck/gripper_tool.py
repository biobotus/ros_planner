#!/usr/bin/python

from deck.deck_module import Coordinate, DeckModule
from protocol.protocol import Protocol, Step

class GripperTool(DeckModule):
    def __init__(self, m_type, coord):
        """
        Constructor
        """
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
        """
        Main function when operator gripper is called from directly from a protocol
        """
        self.steps = []
        description = []

        for instruction in json_instruction['groups']:
            if "move" in instruction:
                val=instruction['move']
                self._parse_move(val, module_dic, self.steps)
                description.append("Moving.".format(val))
            else:
                self.logger.error("unknown operation: {0}".format(instruction))
                pass
        return self.steps, description

    def _parse_move(self, val, module_dic, steps):
        """
        This function move an object from a place to another with the gripper
        """
        #TO DO Add dynamic object

        if val["from_container"] == "TAC":
            self.tac_to_holder(module_dic, steps)

        if val["from_container"] == "vial_holder":
            self.holder_to_tac(module_dic, steps)

        return

    def holder_to_tac(self, module_dic, steps):
        #TO DO Change this funciton to move dynamicly
        vial_holder = module_dic['vial_holder']
        tac = module_dic['TAC']

        opening = 15
        self.gripper_move(vial_holder, tac, [0,0,0],[0,0,0], 0, opening, module_dic, steps)

        return

    def tac_to_holder(self, module_dic, steps):
        #TO DO erase this function
        vial_holder = module_dic['vial_holder']
        tac = module_dic['TAC']

        opening = 15
        self.gripper_move(tac, vial_holder, [0,0,0],[0,0,0], 0, opening, module_dic, steps)

        return

    def gripper_move(self, from_mod, to_mod, from_offset, to_offset, height, opening, module_dic, steps):
        """
        Function to take something with the gripper and move it
        """
        from_coord = self.actual_mod_pos(module_dic, from_mod.get_mod_coordinate())
        to_coord = self.actual_mod_pos(module_dic, to_mod.get_mod_coordinate())

        grab_opening = opening - 9

        cruise_height = height

        steps.append(self.move_pos(Coordinate(from_coord.coord_x+from_offset[0], \
                                              from_coord.coord_y+from_offset[1], \
                                              cruise_height), \
                                   module_dic))

        steps.append(self.gripper_pos(-90,opening))

        steps.append(self.move_pos(Coordinate(from_coord.coord_x+from_offset[0], \
                                              from_coord.coord_y+from_offset[1], \
                                              from_coord.coord_z + from_offset[2]), \
                                   module_dic))


        steps.append(self.gripper_pos(-90,grab_opening))

        steps.append(self.move_pos(Coordinate(from_coord.coord_x+from_offset[0], \
                                              from_coord.coord_y+from_offset[1], \
                                              cruise_height), \
                                   module_dic))

        steps.append(self.move_pos(Coordinate(to_coord.coord_x+to_offset[0], \
                                              to_coord.coord_y+to_offset[1], \
                                              cruise_height), \
                                   module_dic))

        steps.append(self.move_pos(Coordinate(to_coord.coord_x+to_offset[0], \
                                              to_coord.coord_y+to_offset[1], \
                                              to_coord.coord_z + to_offset[2]), \
                                   module_dic))

        steps.append(self.gripper_pos(-90,opening))
        steps.append(self.move_pos(Coordinate(to_coord.coord_x+to_offset[0], \
                                              to_coord.coord_y+to_offset[1], \
                                              cruise_height), \
                                   module_dic))

        steps.append(self.gripper_pos(87,0))
        return

    def gripper_pos(self, wrist, opening):
        """
        Create a message for coordinator
        """
        if self.wrist_pos_max>=int(wrist)>=self.wrist_pos_min and self.opening_pos_max>=int(opening)>=self.opening_pos_min :
            args = {"wrist": float(wrist), "opening": float(opening)}  #, "speed": float(speed)}
            params = {"name": "manip", "args": args}
            step_move = Step({"module_type": self.m_type, "params": params})
            return step_move
        else:
            return -1

    def move_pos(self, coord, module_dic):
        """
        Move the gripper axis to a desired position
        """
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
        """
        Relative position of a module from the gripper
        """
        coord.coord_x = coord.coord_x-self.mod_coord.coord_x
        coord.coord_y = coord.coord_y-self.mod_coord.coord_y
        coord.coord_z = coord.coord_z-self.mod_coord.coord_z
        return coord

