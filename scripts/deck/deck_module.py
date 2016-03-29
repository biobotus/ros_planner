"""
Module contain ModuleManager, Module, ModuleParam
ModuleManager give an interface to store and retrieve modules by their Ids
Module contain ModuleParam wich describe what can be ask to a module.
"""

# cos and sin used in rotation fonction
import math
from protocol.protocol import Step, StepParameter

# using the python ubber logger
import logging


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
        return "(x: {0}; y: {1}; z: {2})".format(self.coord_x, self.coord_y, self.coord_z)

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

    def add_module(self, module, module_id):
        """
        Add a module

        @param module the module to be add
        @param module_id the id of the module
        """
        self.logger.info('Module %s : %s added', module.name, module_id)
        self.modules[module_id] = module

    def get_module(self, module_id):
        """
        Return the module of a given id

        @param module_id the id of the module to be retrieve
        """
        if module_id in self.modules:
            return self.modules[module_id]
        else:
            return -1

    def list_module(self):
        print "Modules  list :"
        for module in self.modules:
            print(self.get_module(module))

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
        @param module_param the parameter the module accept @see ModuleParam
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

        letter = letter - ord('A') + 1
        # add more information on the log error + check for < 0
        if letter > self.nb_line or letter < 1:
            self.logger.error("Error on the line")
            return -1

        if number > self.nb_column or number < 1:
            self.logger.error("Error on the column")
            return -1

        coord_x = self.coord.coord_x + self.well1_offset.coord_x + \
                    (letter-1) * self.well_offset.coord_x
        coord_y = self.coord.coord_y +self.well1_offset.coord_y + \
                    (number-1)*self.well_offset.coord_y

        return Coordinate(coord_x, coord_y, coord_z=0)

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


class ModuleParam():
    """
    Represent a param that can be command (in) or ask (out) to a module.
    """

    def __init__(self, name, p_in, p_out, vmax=0, vmin=0):
        """
        Constructor for ModuleParam.
        @param name the param denomination, temperatur, spin etc..
        @param p_in true if the param is a command, false either.
        @param p_out true if the param is a sensor information, false either.
        @param max the max this param can be command to.
        @param min the min this param can be command to.
        """
        self.name = name
        self.p_in = p_in
        self.p_out = p_out
        self.max = vmax
        self.min = vmin

    def __str__(self):
        """
        Return a string representing the param, its name.
        """
        return self.name
