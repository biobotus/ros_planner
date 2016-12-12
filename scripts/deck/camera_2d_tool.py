#!/usr/bin/python

import pymongo
import time
from deck.deck_module import Coordinate, DeckModule
from protocol.protocol import Protocol, Step

class Camera2DTool(DeckModule):
    def __init__(self, m_type, coord):
        super(Camera2DTool, self).__init__(m_type, coord)
        self.logger.info("BCA initialized")
        self.m_type = m_type
        self.mod_coord = self.get_mod_coordinate()
        self.x_buff = 0
        self.y_buff = 0
        # Colony selection parameters
        self.perimeter_min = -1
        self.perimeter_max = 1e10
        self.excentricity_min = -1
        self.excentricity_max = 1e10
        self.area_min = -1
        self.area_max = 1e10
        self.number_of_colony_d = 0
        self.picking = 0
        self.protocol = None
        self.step = 0
        self.color = '#ffffff'
        self.pick_number = 0
        self.max_cruise_height = 0
        self.client = pymongo.MongoClient()
        self.lid_holder_x_pos = -150
        self.lid_holder_y_pos = 0
        self.lid_holder_z_pos = -5
        self.backlight_z_offset = -10
        self.max_column = 8
        self.backlight_light = 'close'
        self.opening = 45 

    def parse_json(self, json_instruction, module_dic, protocol_name):
        """
        Main function called when a operation is defined in the protocol
        An operation is defined for a specific petri dish so this function
        take care of moving the said petri dish into position and remove the
        lid before engaging in the action demanded by the protocol
        """

        self.steps = []
        description= []
        self.petri_mod = module_dic[json_instruction['name']]
        self.gripper_mod = module_dic["gripper"]
        self.petri_coord = self.actual_mod_pos(module_dic, self.petri_mod.get_mod_coordinate())
        self.backlight_mod = module_dic["backlight_module"]
        self.backlight_coord = self.actual_mod_pos(module_dic, self.backlight_mod.get_mod_coordinate())
        trash_mod = module_dic["trash"]
        self.dump_coord  = self.actual_mod_pos(module_dic, trash_mod.get_mod_coordinate())
        self.protocol = protocol_name
        self.cruise_height = self.max_cruise_height
        self.instructions = json_instruction['groups']
        self.pipette_mod = module_dic["pipette_s"]
        self.backlight_abs_coord = self.backlight_mod.get_mod_coordinate()

        self.backlight = json_instruction['light']

        if json_instruction['groups'] == None:
            return self.steps, description

        self.gripper_mod.gripper_move(self.petri_mod, self.backlight_mod, [0, 0, 0], \
                                      [self.lid_holder_x_pos, self.lid_holder_y_pos, \
                                      self.lid_holder_z_pos+self.backlight_z_offset], \
                                      self.max_cruise_height, self.opening, module_dic, self.steps)

        self.gripper_mod.gripper_move(self.petri_mod, self.backlight_mod, [0, 0, 0], \
                                      [0, 0, self.backlight_z_offset], \
                                      self.max_cruise_height, self.opening, module_dic, self.steps)

        for instruction in json_instruction['groups']:
            if instruction['action'] == 'analyze':
                self._parse_analyze(module_dic)
                description.append("Analyzing the {0} petri dish.".format(json_instruction['name']))

            elif instruction['action'] == 'autopick':
                self.module_dest = instruction['to'].split('/')
                self.number_of_colony_d = instruction['number']
                self._parse_autopick(instruction['criterias'], module_dic)
                description.append("Picking {0}: {1} colonies from {2} Petri dish to {3}.".format(self.pick_number, \
                                       self.number_of_colony_d, json_instruction['name'], instruction['to']))
                self.pick_number += 1
            else:
                self.logger.error("unknown operation: {0}".format(instruction))
                pass

        self.gripper_mod.gripper_move(self.backlight_mod, self.petri_mod, \
                                      [0,0, self.backlight_z_offset], \
                                      [0,0,0], self.max_cruise_height, self.opening,\
                                      module_dic, self.steps)

        self.gripper_mod.gripper_move(self.backlight_mod, self.petri_mod, \
                                      [self.lid_holder_x_pos, self.lid_holder_y_pos, \
                                      self.lid_holder_z_pos + self.backlight_z_offset], \
                                      [0,0,0], self.max_cruise_height, self.opening, module_dic, self.steps)

        self.backlight = 'close'
        data = []
        data.append(self.perimeter_min)
        data.append(self.perimeter_max)
        data.append(self.excentricity_min)
        data.append(self.excentricity_max)
        data.append(self.area_min)
        data.append(self.area_max)
        data.append(self.color)
        data.append(self.number_of_colony_d)
        data.append(self.picking)
        data.append(self.protocol)
        data.append(self.step)
        data.append(self.pick_number)
        data.append(self.backlight)
        self.steps.append(self.bca_analysis(data, module_dic))

        return self.steps, description

    def _parse_analyze(self, module_dic):
        """
        This function analyse a petri dish with watershed
        """

        data = []
        data.append(self.perimeter_min)
        data.append(self.perimeter_max)
        data.append(self.excentricity_min)
        data.append(self.excentricity_max)
        data.append(self.area_min)
        data.append(self.area_max)
        data.append(self.color)
        data.append(self.number_of_colony_d)
        data.append(self.picking)
        data.append(self.protocol)
        data.append(self.step)
        data.append(self.pick_number)
        data.append(self.backlight)

        self.steps.append(self.gripper_mod.move_pos(Coordinate(self.backlight_coord.coord_x, \
                                                               self.backlight_coord.coord_y, \
                                                               self.backlight_coord.coord_z), \
                                                    module_dic))

        self.steps.append(self.bca_analysis(data, module_dic))

        self.steps.append(self.gripper_mod.move_pos(Coordinate(self.backlight_coord.coord_x, \
                                                               self.backlight_coord.coord_y, \
                                                               self.cruise_height), \
                                                    module_dic))

        return self.steps

    def _parse_autopick(self, pick_json, module_dic):
        """
        This function do an autopicking manipulation on the petri dish.
        """

        db = self.client[self.protocol]
        pick_num = "picking_{}".format(self.pick_number)
        colonies_to_pick = list(db.colonies.find({'step': self.step, 'operation': pick_num, 'selected': 1}))
        print(colonies_to_pick)

        characteristics = {'step': self.step, 'pick_num': pick_num, \
                           'nb_col': self.number_of_colony_d}
        char_crit = []

        for criteria in pick_json:
            if criteria['category'] == 'color':
                print(criteria['color'])
                self.color = criteria['color']
                char_crit.append(('Color', '<span class="glyphicon glyphicon-stop" aria-hidden=true style="color:{0}"></span>'.format(self.color)))

            if criteria['category'] == 'size':
                self.area_max = criteria['maximum']
                self.area_min = criteria['minimum']
                char_crit.append(('Size', "> {0} mm<sup>2</sup> and < {1} mm<sup>2</sup>".format(self.area_min, self.area_max)))

            if criteria['category'] == 'segmentation':
                self.picking = not(criteria['segment'])
                char_crit.append(('Accept touching colonies', 'No' if self.picking else 'Yes'))

        characteristics['criterias'] = char_crit
        db.picking.insert_one(characteristics)

        self.picking = True
        data = []
        data.append(self.perimeter_min)
        data.append(self.perimeter_max)
        data.append(self.excentricity_min)
        data.append(self.excentricity_max)
        data.append(self.area_min)
        data.append(self.area_max)
        data.append(self.color)
        data.append(self.number_of_colony_d)
        data.append(self.picking)
        data.append(self.protocol)
        data.append(self.step)
        data.append(self.pick_number)
        data.append(self.backlight)

        self.steps.append(self.gripper_mod.move_pos(Coordinate(self.backlight_coord.coord_x, \
                                                               self.backlight_coord.coord_y, \
                                                               self.backlight_coord.coord_z), \
                                                    module_dic))

        self.steps.append(self.bca_analysis(data, module_dic))

        self.steps.append(self.gripper_mod.move_pos(Coordinate(self.backlight_coord.coord_x, \
                                                               self.backlight_coord.coord_y, \
                                                               self.cruise_height), \
                                                    module_dic))

        to_mod = module_dic[str(self.module_dest[0])]
        initial_letter = self.module_dest[1][0]
        initial_number = self.module_dest[1][1:]
        self.x_buff = 0
        self.y_buff = 0

        for i in range(self.number_of_colony_d):
            number = int(initial_number) - 1 + self.x_buff
            letter = ord(initial_letter) - ord('A') + self.y_buff

            self.pipette_mod.autopick(Coordinate(self.backlight_abs_coord.coord_x, \
                                                 self.backlight_abs_coord.coord_y, \
                                                 self.backlight_abs_coord.coord_z), \
                                      to_mod, number, letter, module_dic, self.steps)
            self.y_buff += 1
            if self.y_buff == self.max_column:
                self.x_buff += 1
                self.y_buff = 0
        return

    def actual_mod_pos(self, module_dic, coord):
        """
        This function defines the relative position of a module from the 2D camera
        """

        coord.coord_x = coord.coord_x-self.mod_coord.coord_x
        coord.coord_y = coord.coord_y-self.mod_coord.coord_y
        coord.coord_z = coord.coord_z-self.mod_coord.coord_z
        return coord

    def bca_analysis(self, data, module_dic):
        """
        This function create a BCA analysis message for coordinator
        """
        args =  {"perimeter_min": float(data[0]), "perimeter_max": float(data[1]), \
                 "excentricity_min": float(data[2]),"excentricity_max": float(data[3]), \
                 "area_min": float(data[4]), "area_max": float(data[5]), \
                 "color": str(data[6]), \
                 "number_of_colony_d": int(data[7]), "picking": bool(data[8]), \
                 "protocol": str(data[9]), "step": int(data[10]), \
                 "picking_number": int(data[11]), "backlight_color": str(data[12])}

        params = {"name": "analysis", "args": args}
        step_move = Step({"module_type": self.m_type, "params": params})

        return step_move

    def calibrate_2d(self, module_dic):
        """
        To be implemented
        """
        pass

class BackLightModule(DeckModule):
    def __init__(self, name, coord):
            super(BackLightModule, self).__init__(name, coord)
