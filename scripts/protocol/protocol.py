#!/usr/bin/python

import collections
import json
import logging
import pymongo
import threading
import time

# Get MongoDB client
client = pymongo.MongoClient()

def load_protocol_from_json(json_string, module_manager):
    data = json.loads(json_string)
    protocol = Protocol()

    instructions = data['instructions']
    labware_description = data['refs']
    logger = logging.getLogger()

    # We build a dictionary with all the modules/labwares used on the protocol
    # to link the name they are refered by and their id on the deck
    for labware in labware_description:
        if not labware['name'] in module_manager.modules:
            mod = module_manager.add_module(labware)

    for instruction in instructions:
        if 'op' in instruction and instruction['op'] in module_manager.modules:
            mod = module_manager.modules[instruction['op']]
            steps = mod.parse_json(instruction, module_manager.modules)
            protocol.add_steps(steps)

        else:
            logger.error("Instruction error : wrong operator or groups")

    return protocol


class Protocol:
    """
    Protocol describe a Protocol of instructions for multiples modules
    Protocol are a named suite of instructions grouped as steps.
    """

    def __init__(self):
        self.name = "protocol_{}".format(int(time.time()))
        self.db = client[self.name]
        self.current_step = None
        self.steps = collections.deque()
        self.modules = []
        self.logger = logging.getLogger(__name__)
        self.logger.info("Creation of Protocol : %s", self.name)

    def __str__(self):
        """
        Return a string representing the protocol
        """
        return self.name

    def add_steps(self, steps):
        """
        Add a step to the Protocol
        @param step the step to be add
        """
        for step in steps:
            self.logger.info("adding step to Protocol : {0}".format(self.name))
            self.steps.append(step)

    def get_module_list(self):
        """
        Return the list of modules active in the Protocol
        """
        modules = []
        for step in self.steps:
            modules += step.get_module_list()

        return list(set(modules))

    def start(self):
        """
        Start the first step of the Protocol
        """
        self.start_next_step()
        self.logger.info("Protocol : " + self.name + " is started")

    def start_next_step(self):
        """
        Start the Protocol with the first step
        """
        if self.steps:
            self.logger.info("Starting next step of Protocol :" + self.name)
            self.current_step = self.steps.popleft()
            #mettre ici le code pour envoyer la commande
        else:
            self.logger.info("No more steps for : " + self.name)

class Step:
    """
    A step is the core component of a Protocol.
    A step is the combinaison of a stop condition and a set of parameters to be
    send to modules
    """

    def __init__(self, step_dict):
        self.step_dict = step_dict

    def __str__(self):
        return str(self.step_dict)

    def add_parameter(self, step_param):
        """
        Add a parameter to the step
        @param self
        @para step_param the param to be add
        """
        #self.logger.info("Added a parameter to the step : " + step_param.name + " @ "
        #         + str(step_param.value))
        self.params.append(step_param)

    def get_module_list(self):
        """
        Get the list of module use in the step
        @return a list of unique module
        """
        module_list = []
        for param in self.params:
            if not param.module in module_list:
                module_list.append(param.module)
        return module_list


class StepParameter:
    """
    StepParameter, merain component of a step, a step parameter is a parameter
    to be enforced on a module.
    """

    def __init__(self, module, name="", value=0):
        """
        Constructor
        @param module the module to force the parameter to.
        @param name the parameter name, temperature for exemaple
        @param value the value to be set for a given parameter name
        """
        self.module = module
        self.name = name
        self.value = value

    def __str__(self):
        return "{0}: {1} @ {2}".format(self.module, self.name, self.value)

    def is_match(self, module, name, value):
        """
        Return true if the step parameter match a given for a module
        """
        return self.module == module and self.name == name and self.value == value

