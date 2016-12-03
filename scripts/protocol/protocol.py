#!/usr/bin/python

import collections
import fallocate
import json
import logging
import pymongo
import threading
import time

# Get MongoDB client
client = pymongo.MongoClient()

def alloc_db(name):
    """
    Allocate disk space for MongoDB manually to prevent using
    default sizes of 32 MB or more.
    """

    for i in ['ns', '0', '1']:
        with open("/data/db/{0}.{1}".format(name, i), 'w+b') as f:
            fallocate.posix_fallocate(f, 0, 1048576*3)

def mapping_3d_protocol(module_manager, data):

    protocol = Protocol(data)
    mod = module_manager.modules['3d_camera']
    steps = mod.mapping_3d(module_manager.modules)
    protocol.add_steps(steps)

    return protocol

def load_protocol_from_json(json_string, module_manager):
    data = json.loads(json_string)
    protocol = Protocol(data)
    db = protocol.db
    db.protocol.insert_one({'protocol': protocol.data})

    instructions = protocol.data['instructions']
    labware_description = protocol.data['refs']
    logger = logging.getLogger()

    # We build a dictionary with all the modules/labwares used on the protocol
    # to link the name they are refered by and their id on the deck
    for labware in labware_description:
        if not labware['name'] in module_manager.modules:
            mod = module_manager.add_module(labware)

    number = 1
    for instruction in instructions:
        if 'op' in instruction and instruction['op'] in module_manager.modules:
            mod = module_manager.modules[instruction['op']]
            if instruction['op'] == '2d_camera':
                steps, description = mod.parse_json(instruction, module_manager.modules, protocol.name)
            else:
                steps, description = mod.parse_json(instruction, module_manager.modules)

            protocol.add_steps(steps)
            db.steps.insert_one({'number': number, 'description': '\n'.join(description)})
            number += 1

        else:
            logger.error("Instruction error : wrong operator or groups")

    return protocol


class Protocol:
    """
    Protocol describe a Protocol of instructions for multiples modules
    Protocol are a named suite of instructions grouped as steps.
    """

    def __init__(self, data):
        self.name = "protocol_{}".format(int(time.time()))
        alloc_db(self.name)
        self.data = data
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
        new_step = collections.deque()
        for step in steps:
            self.logger.info("adding step to Protocol : {0}".format(self.name))
            new_step.append(step)

        self.steps.append(new_step)

class Step:
    """
    A step is the core component of a Protocol.
    """

    def __init__(self, step_dict):
        self.step_dict = step_dict

    def __str__(self):
        return str(self.step_dict)

