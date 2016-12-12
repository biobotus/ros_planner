#!/usr/bin/python

from deck.deck_module import Coordinate, DeckModule
from protocol.protocol import Protocol, Step

class Camera3DTool(DeckModule):
    """
    Initialise the Camera3DTool class.
    Input:  DeckModule
    Output: returns nothing
    """
    def __init__(self, m_type, coord):
        """
        Initialise the class.
        Input:
        Output:
        """
        super(Camera3DTool, self).__init__(m_type, coord)
        self.logger.info("Gripper initialized")
        self.m_type = m_type
        self.mod_coord = self.get_mod_coordinate()
        self.x_buff = 0
        self.y_buff = 0
        # Constant used for simple pipette across the code
        self.nb_x = 3
        self.nb_y = 3
        self.delta_x = 400
        self.delta_y = 292
        self.offset_x = 100
        self.offset_y = 0

    def mapping_3d(self, module_dict):
        """
        mapping_3d function move from square on deck to another.
        Input: Module_dict
        Output: steps [platform movements]
        """
        self.steps = []
        for nx in range(self.nb_x):
            for ny in range(self.nb_y):
                pos_x = (nx*self.delta_x)+self.offset_x
                pos_y = (ny*self.delta_y)+self.offset_y
                coord = [pos_x, pos_y, 0]
                self.steps.append(self.move_pos(coord, module_dict))
                self.steps.append(self.aquisition_3d(nx,ny))
        return self.steps

    def aquisition_3d(self, nx, ny):
        """
        Input: nx and ny (matrix adress of the square being analyzed)
        Output: step_move
        """
        args = {"nx": float(nx), "ny": float(ny)}
        params = {"name": "manip", "args": args}
        step_move = Step({"module_type": self.m_type, "params": params})
        return step_move

    def move_pos(self, coord, module_dict):
        """
        Input: coord, module_dict
        Output: step_move
        """

        if int(coord[1])<0 or int(coord[1])<0 or int(coord[2]) < 0:
            print("Negative coord x: {} y: {} z: {}"\
                .format(coord.coord_x, coord.coord_y, coord.coord_z))
            self.logger.error("Received a negative coordinate x: {} y: {} z: {}"\
                .format(coord.coord_x, coord.coord_y, coord.coord_z))
            return -1
        elif int(coord[0])>1000 or int(coord[1])>585:
            print("Over coord x: {} y: {} z: {}"\
                .format(coord.coord_x, coord.coord_y, coord.coord_z))
            self.logger.error("Received a too far coord x: {} y: {} z: {}"\
                .format(coord.coord_x, coord.coord_y, coord.coord_z))
            return -1
        else:
            args =  {"x": coord[0], "y": coord[1], "z": coord[2]}
            params = {"name": "pos", "args": args}
            step_move = Step({"module_type": self.m_type, "params": params})

        return step_move

    def actual_pos(self, module_dict, coord):
        """
        Input: module_dict, coord
        Output: coord
        """
        coord.coord_x = coord.coord_x-self.mod_coord.coord_x
        coord.coord_y = coord.coord_y-self.mod_coord.coord_y
        coord.coord_z = coord.coord_z-self.mod_coord.coord_z
        return coord
