#!/usr/bin/python

"""
Module contain ModuleManager, Module
ModuleManager give an interface to store and retrieve modules by their Ids
"""
import json
import logging
import math
from protocol.protocol import Step
import pymongo

client = pymongo.MongoClient()
biobot = client['biobot']

class Coordinate():
    """
    coordinate represents a position (x,y,z) inside the deck. This class offer
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
        """
        Add a module in the planner dict by fetching the information in the database
        """

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
        """
        Add a tool in the planner dict by fetching the information in the database
        """

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
        """
        Create a large container labware
        """
        return Large_Container(m_name, coord)

    def add_small_tip_holder(self, m_name, coord):
        """
        Create a small tip holder labware
        """
        return Small_Tip_Holder(m_name, coord)

    def add_medium_tip_holder(self, m_name, coord):
        """
        Create a medium tip holder labware
        """
        return Medium_Tip_Holder(m_name, coord)

    def add_large_tip_holder(self, m_name, coord):
        """
        Create a large tip holder labware
        """
        return Large_Tip_Holder(m_name, coord)

    def add_centrifuge_vial_holder(self, m_name, coord):
        """
        Create a centrifuge vial holder labware
        """
        return Centrifuge_Vial_Holder(m_name, coord)

    def add_multiwell_plate(self, m_name, coord):
        """
        Create a multiwell plate labware
        """
        return Multiwell_Plate(m_name, coord)

    def add_trash(self, m_name, coord):
        """
        Create a thrash
        """
        self.logger.info("Add trash module")
        return Trash_bin(m_name, coord)

    def add_pipette_s(self, m_type, coord):
        """
        Create a single pipette tool
        """
        return PipetteModule(m_type, coord)

    def add_pipette_m(self, m_type, coord):
        """
        Create a multi pipette tool
        """
        return PipetteModule(m_type, coord)

    def add_gripper(self, m_type, coord):
        """
        Create a gripper tool
        """
        return GripperTool(m_type, coord)

    def add_3d_camera(self, m_type, coord):
        """
        Create a 3D camera tool
        """
        return Camera3DTool(m_type, coord)

    def add_2d_camera(self, m_type, coord):
        """
        Create a 2D camera tool
        """
        return Camera2DTool(m_type, coord)

    def add_petri_dish(self, m_type, coord):
        """
        Create a petri dish labware
        """
        return PetriDish(m_type, coord)

    def add_backlight_module(self, m_type, coord):
        """
        Create a backlight module
        """
        return BackLightModule(m_type, coord)

    def add_tac(self, m_type, coord):
        """
        Create a TAC module
        """
        return TAC(m_type, coord)

    def add_vial_holder(self, m_type, coord):
        """
        Create a vial holder labware
        """
        return Vial_Holder(m_type, coord)

    def add_safety_tip(self, m_type, coord):
        """
        Create a safety tip module
        """
        return Safety_Tip(m_type, coord)

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
        self.diameter = 0
        self.well1_offset = Coordinate(0, 0, 0)
        self.well_offset = Coordinate(0, 0, 0)
        self.logger = logging.getLogger(__name__)

    def __str__(self):
        """
        Return a string representing the module (the name)
        """
        return "module : " + self.name

    def set_mod_diameter(self, d):
        """
        Set a labware diameter
        """
        self.diameter = d

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
        Return a well coordinate, according to the global referential
        (robot top left corner)
        """
        if number > self.nb_column - 1 or number < 0:
            err = "Error on the line of module {0}: Got letter {1}, which is not in [0, {2}[ interval.".format(self.name, number, self.nb_line)
            print(err)
            raise Exception(err)

        if letter > self.nb_line-1 or letter < 0:
            err = "Error on the column of module {0}: Got letter {1}, which is not in [0, {2}[ interval.".format(self.name, letter, self.nb_column)
            print(err)
            raise Exception(err)

        mod_coord = self.get_mod_coordinate()
        coord_x = self.coord.coord_x + self.well1_offset.coord_x + \
                                   number * self.well_offset.coord_x
        coord_y = self.coord.coord_y +self.well1_offset.coord_y + \
                                   letter *self.well_offset.coord_y

        return Coordinate(coord_x, coord_y, mod_coord.coord_z)

    def get_mod_coordinate(self):
        """
        Return a labware, module or tool absolute coordinate
        """
        return Coordinate(self.coord.coord_x, self.coord.coord_y, self.coord.coord_z)

    def get_mod_diameter(self):
        """
        Return a labware, module or tool diameter
        """
        return self.diameter

# To prevent cyclic import
from deck.camera_2d_tool import Camera2DTool, BackLightModule
from deck.camera_3d_tool import Camera3DTool
from deck.gripper_tool import GripperTool
from deck.pipette_module import PipetteModule
from deck.labware_module import Trash_bin, Small_Tip_Holder, Medium_Tip_Holder, \
                                Large_Tip_Holder, Centrifuge_Vial_Holder, \
                                Multiwell_Plate, Large_Container, PetriDish, \
                                TAC, Vial_Holder, Safety_Tip
