#!/usr/bin/python

"""
Module contain ModuleManager, Module
ModuleManager give an interface to store and retrieve modules by their Ids
"""
import json
import logging
import math
from protocol.protocol import Step, StepParameter
import pymongo

client = pymongo.MongoClient()
biobot = client['biobot']

class Coordinate():
    """
    coordinate represente a position (x,y,z) inside the deck. This class offer
    tools to translate and rotate those position.
    """

    def __init__(self, coord_x, coord_y, coord_z):
        """
        Constructor for coordinate
        @param coord_x the x coordinate
        @param coord_y the y coordinate
        @param coord_z the z coordinate
        """
        self.coord_x = coord_x
        self.coord_y = coord_y
        self.coord_z = coord_z

    def rotate_z(self, angle, axe):
        """
        Rotate the position around an axe
        @param axe the coordinate of the axe for the rotation
        @param angle the angle for the rotation
        """
        coord_x = self.coord_x - axe.coord_x
        coord_y = self.coord_y - axe.coord_y
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)

        self.coord_x = axe.coord_x + coord_x * cos_angle - coord_y * sin_angle
        self.coord_y = axe.coord_y + coord_x * sin_angle + coord_y * cos_angle

        self.coord_x = round(self.coord_x, 2)
        self.coord_y = round(self.coord_y, 2)

    def translate_x(self, distance):
        """
        Translate the coordinate along the axe X
        @param distance the distance to be translate
        """
        self.coord_x += distance

    def translate_z(self, distance):
        """
        Translate the coordinate along the axe z
        @param distance the distance to be translate
        """
        self.coord_z += distance

    def translate_y(self, distance):
        """
        Translate the coordinate along the axe Y
        @param distance the distance to be translate
        """
        self.coord_y += distance

    def __str__(self):
        """
        Return a string representing the coordinate
        """
        return "(x: {0}; y: {1}; z: {2})".format(self.coord_x, self.coord_y, \
                                                               self.coord_z)

    def __eq__(self, other):
        """
        Overloading of the equal function to compare equity of coordone
        """
        return self.coord_x == other.coord_x and self.coord_y == other.coord_y and \
            self.coord_z == other.coord_z


class DeckManager():
    """
    The module manager handles modules,labware and their Ids.
    """

    def __init__(self):
        """
        Constructor
        """
        self.modules = {}
        self.logger = logging.getLogger(__name__)

    def add_module(self, labware):
        item = biobot.deck.find_one({'name': labware['name'], \
                                     'type': labware['type'], \
                                     'validated': True})
        if item:
            parameters = [self, item['name'], \
                          Coordinate(item['valid_x'], \
                                     item['valid_y'], \
                                     item['valid_z'])]
            try:
                mod = getattr(self.__class__, 'add_{0}'.format(labware['type']))(*parameters)
                self.modules[labware['name']] = mod
                self.logger.info('Module %s : %s added', labware['type'], labware['name'])
                return True
            except AttributeError as e:
                print("Item {} is not yet implemented. Error: {}".format(labware['type'], e))
                return False

        else:
            print("An item in the reference section of the protocol is not on the deck: {}".format(labware))
            return False

    def add_tools(self):
        with open('/home/ubuntu/biobot_web/tools_conf.json', 'r') as f:
            tools = json.load(f)

        for tool in tools:
            parameters = [self, tool['type'], \
                          Coordinate(tool['offset_x'], \
                                     tool['offset_y'], \
                                     tool['offset_z'])]
            try:
                mod = getattr(self.__class__, 'add_{0}'.format(tool['type']))(*parameters)
                self.modules[tool['type']] = mod
                self.logger.info("Tool added: {}".format(tool['type']))
            except AttributeError as e:
                print("Tool {} is not yet implemented. Error: {}".format(tool['type'], e))

    def add_large_container(self, m_name, coord):
        return Large_Container(m_name, coord)

    def add_small_tip_holder(self, m_name, coord):
        return Small_Tip_Holder(m_name, coord)

    def add_medium_tip_holder(self, m_name, coord):
        return Medium_Tip_Holder(m_name, coord)

    def add_large_tip_holder(self, m_name, coord):
        return Large_Tip_Holder(m_name, coord)

    def add_centrifuge_vial_holder(self, m_name, coord):
        return Centrifuge_Vial_Holder(m_name, coord)

    def add_multiwell_plate(self, m_name, coord):
        return Multiwell_Plate(m_name, coord)

    def add_tac(self, m_name, coord):
        self.logger.info("Add tac module")
        return TacModule(m_name, coord)

    def add_trash(self, m_name, coord):
        self.logger.info("Add trash module")
        return Trash_bin(m_name, coord)

    def add_pipette_s(self, m_type, coord):
        return PipetteModule(m_type, coord)

    def add_pipette_m(self, m_type, coord):
        return PipetteModule(m_type, coord)

    def add_gripper(self, m_type, coord):
	return GripperTool(m_type, coord)

    def add_3d_camera(self, m_type, coord):
        return Camera3DTool(m_type, coord)

class DeckModule(object):
    """
    This class represent an entity of a module present on the deck
    """

    def __init__(self, name, coord):
        """ Constructor for module
        @param name The name of the module
        @param coord the coordinate of the module on the deck, top left
           relative to the robot top left corner.
        """
        self.name = name
        self.params = []
        self.coord = coord
        self.nb_line = 0
        self.nb_column = 0
        self.well1_offset = Coordinate(0, 0, 0)
        self.well_offset = Coordinate(0, 0, 0)
        self.logger = logging.getLogger(__name__)

    def __str__(self):
        """
        Return a string representing the module (the name)
        """
        return "module : " + self.name


    def add_parameter(self, module_param):
        """
        Add a parameter to the module.
          Module parameter represent what the module can recieve as instruction
          or transmit as information
        @param module_param the parameter the module accept
        """
        self.logger.info("Added a parameter to the module : " + self.name)
        self.params.append(module_param)

    def set_well_layout(self, nb_line, nb_column, first_offset, offset):
        """
        Set how the wells are repartited on the plate/module.
        @param nb_line the number of well lines
        @param nb_column the number of well column
        @param x_offset the distance between two columns in mm
        @param y_offset the distance between two lines in mm
        """
        self.nb_line = nb_line
        self.nb_column = nb_column
        self.well1_offset = first_offset
        self.well_offset = offset

    def get_well_coordinate(self, letter, number):
        """
        Return a well coordinate, acording to the global referential
        (robot top left corner)
        """
        # add more information on the log error + check for < 0
        if letter > self.nb_line-1 or letter < 0:
            self.logger.error("Error on the line")
            print "Error on the line"
            return -1

        if number > self.nb_column-1 or number < 0:
            self.logger.error("Error on the column")
            print "Error on the column"
            return -1

        mod_coord = self.get_mod_coordinate()
        coord_x = self.coord.coord_x + self.well1_offset.coord_x + \
                    (number) * self.well_offset.coord_x
        coord_y = self.coord.coord_y +self.well1_offset.coord_y + \
                    (letter)*self.well_offset.coord_y
        return Coordinate(coord_x, coord_y, mod_coord.coord_z)

    def get_mod_coordinate(self):
        return Coordinate(self.coord.coord_x, self.coord.coord_y, self.coord.coord_z)

    def parse_json(self, json_instruction, module_dic):
        """
        Parse a json structure concerning this module.
        """
        stop_condition = StepParameter(module = self,
                             name = json_instruction['stop']['condition'],
                             value = json_instruction['stop']['value'])

        step = Step(stop_condition)
        for param in self.params :
            mod_value = 2  # valeur par default, a definir?
            if param.p_in and param.name in json_instruction:
                mod_value = json_instruction[param.name]

            if param.p_in:
                step.add_parameter(StepParameter(module=self,
                                                 name=param.name,
                                                 value=mod_value))
        return [step,]

# To prevent cyclic imports
from deck.camera_3d_tool import Camera3DTool
from deck.gripper_tool import GripperTool
from deck.pipette_module import PipetteModule
from deck.labware_module import Trash_bin, Small_Tip_Holder, Medium_Tip_Holder, \
                                Large_Tip_Holder, Centrifuge_Vial_Holder, \
                                Multiwell_Plate, Large_Container
