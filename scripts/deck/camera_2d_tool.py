#!/usr/bin/python

import pymongo

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
        self.perimeter_min = 0
        self.perimeter_max = 0
        self.excentricity_min = 0
        self.excentricity_max = 0
        self.area_min = 0
        self.area_max = 0
        self.number_of_colony_d = 0
        self.picking = 0
        self.protocol = None
        self.step = 0
        self.color = []
        self.pick_number = 0
        self.max_cruise_height = 100
        self.client = pymongo.MongoClient()

    def parse_json(self, json_instruction, module_dic, protocol_name):
        self.steps = []
        description= []
        self.petri_mod = module_dic[json_instruction['name']]
        self.gripper_mod = module_dic["gripper"]
        self.backlight_mod = module_dic["backlight_module"]
        self.petri_coord = self.actual_mod_pos(module_dic, self.petri_mod.get_mod_coordinate())
        trash_mod = module_dic["trash"]
        self.dump_coord  = self.actual_mod_pos(module_dic, trash_mod.get_mod_coordinate())
        self.protocol = protocol_name
        self.cruise_height = self.dump_coord.coord_z-self.max_cruise_height

        if json_instruction['groups'] == None:
            return self.steps, description


        self.gripper_mod.gripper_move(self.petri_mod, self.backlight_mod, self.max_cruise_height, module_dic, self.steps)
        for instruction in json_instruction['groups']:
            if instruction['action'] == "analyze":
                self._parse_analyze(module_dic)
                description.append("Analyzing the {0} petri dish.".format(json_instruction['name']))
            elif "autopick" in instruction:
                self._parse_autopick(instruction['autopick'], module_dic)
            else:
                self.logger.error("unknown operation: {0}".format(instruction))
                pass

        self.gripper_mod.gripper_move(self.backlight_mod, self.petri_mod, self.max_cruise_height, module_dic, self.steps)

        return self.steps, description

    def _parse_analyze(self, module_dic):

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
        #Moving gripper
        #self.gripper_mod.gripper_move(self.petri_mod, self.backlight_mod, self.max_cruise_height, module_dic, self.steps)
        #Moving camera
        self.steps.append(self.gripper_mod.move_pos(Coordinate(self.petri_coord.coord_x, self.petri_coord.coord_y, self.petri_coord.coord_z),module_dic))

        self.steps.append(self.bca_analysis(data, module_dic))

        self.steps.append(self.gripper_mod.move_pos(Coordinate(self.petri_coord.coord_x, self.petri_coord.coord_y, self.cruise_height),module_dic))

        return self.steps

    def _parse_autopick(self, var, module_dic):
        db = self.client[self.protocol]
        pick_num = "picking_{}".format(self.pick_number)
        colonies_to_pick = list(db.colonies.find({'step': self.step, 'operation': pick_num, 'selected': 1}))
        print(colonies_to_pick)
        for colony in colonies_to_pick:
            x, y = colony['y'], colony['x']
            print("x: {}, y: {}".format(x,y))


        return


    def actual_mod_pos(self, module_dic, coord):
        coord.coord_x = coord.coord_x-self.mod_coord.coord_x
        coord.coord_y = coord.coord_y-self.mod_coord.coord_y
        coord.coord_z = coord.coord_z-self.mod_coord.coord_z
        return coord

    def bca_analysis(self, data, module_dic):

        args =  {"perimeter_min": float(data[0]), "perimeter_max": float(data[1]), \
                 "excentricity_min": float(data[2]),"excentricity_max": float(data[3]), \
                 "area_min": float(data[4]), "area_max": float(data[5]), \
                 "color": data[6], \
                 "number_of_colony_d": int(data[7]), "picking": bool(data[8]), \
                 "protocol": str(data[9]), "step": int(data[10]), "picking_number": int(data[11])}
        params = {"name": "analysis", "args": args}
        step_move = Step({"module_type": self.m_type, "params": params})

        return step_move

    def bca_autopick(self, var, module_dic):

        #args =  {""}
        params = {"name": "analysis", "args": args}
        step_move = Step({"module_type": self.m_type, "params": params})

        return step_move


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


class BackLightModule(DeckModule):
    def __init__(self, name, coord):
        super(BackLightModule, self).__init__(name, coord)
