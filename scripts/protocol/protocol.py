from collections import deque
import time
import json
from threading import Timer
import logging


def param_to_frame(param):
    return {
        "density": "D",
        "time": "T",
        "temperature": "K",
        "spin": "A"
    }[param]


def load_protocol_from_json_file(json_file, module_manager):

    with open(json_file, 'r') as json_data:
        data = json.load(json_data)
        return load_protocol_from_json(data, module_manager)


def load_protocol_from_json_string(json_string, module_manager):

    data = json.loads(json_string)

    return load_protocol_from_json(data, module_manager)


def load_protocol_from_json(json_data, module_manager):
    #the new protocol
    protocol = Protocol(name="Jsonprotocol2")
    #print json.dumps(json_data, sort_keys=True, indent=4)

    #print json.dumps(json_data['instructions'], sort_keys=True, indent=4)

    instructions = json_data['instructions']
    labware_description = json_data['refs']
    logger = logging.getLogger()
    module_dict = {}

    # We build a dictionary with all the modules/labwares used on the protocol
    # to link the name they are refered by and their id on the deck
    for labware in labware_description:
        mod = module_manager.get_module(labware_description[labware]["id"])
        if mod == -1:
            # raise an error
            # A module on the json is not on the deck
            print "A module on the json refs section is not on the deck : "
            print(labware_description[labware]["id"])
            logger.error("A module on the json refs section is not on the deck")
        else:
            module_dict[labware] = mod
    print(module_dict)

    for instruction in instructions:
        if 'op' in instruction and instruction['op'] in labware_description and 'groups' in instruction:
            mod = module_dict[instruction['op']]
            steps = mod.parse_json(instruction['groups'], module_dict)
            protocol.add_steps(steps)

        else:
            logger.error("Instruction error : wrong operator or groups")

    return protocol


class Protocol:
    """
    Protocol describe a Protocol of instructions for multiples modules
    Protocol are a named suit of instructions grouped as steps. (doc a modifier)
    """

    def __init__(self, name = "generic_name"):
        """
        Constructor of Protocol

        @param self
        @param name the name of the Protocol, default to "generic_name"
        """
        self.name = name
        self.current_step = None
        self.steps = deque()
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

    def on_param_received(self, module, name, value):
        """
        Act on the reception of a message.
        For now we only check if the message match a stop condition
        @param module the module sending the message
        @param name name of the parameter received
        @param value value received for the named parameter
        """
        self.logger.info("Protocol "+ self.name + " received : " + name + " @ " +
                str(value) + " from module : " + str(module))
        if self.current_step.condition.is_match(module, name, value):
            self.start_next_step()

    def on_step_started_received(self, module):
        """
        Act on the reception of a module step started.
        """
        if self.current_step.condition.module == module:
            self.current_step.started()
            if self.current_step.condition.name == "time":
                self.logger.info(time.time())
                stop_value = self.current_step.condition.value
                Timer(stop_value, self.start_next_step, ()).start()


class Step:
    """
    A step is the core component of a Protocol.
    A step is the combinaison of a stop condition and a set of parameters to be
    send to modules
    """

    def __init__(self, condition):
        """
        Constructor of step taking the stop condition
        @param condition a step param used as stop condition
        """
        self.condition = condition
        self.params = []
        self.logger = logging.getLogger(__name__)

    def __str__(self):
        string = "Stop : \n"
        string += str(self.condition)+"\n"
        string += "Parameters : \n"
        for param in self.params:
            string += "\t" + str(param) + "\n"

    def add_parameter(self, step_param):
        """
        Add a parameter to the step
        @param self
        @para step_param the param to be add
        """
        self.logger.info("Added a parameter to the step : " + step_param.name + " @ "
                 + str(step_param.value))
        self.params.append(step_param)

    def started(self):
        """
        Function when the Step is signaled as started by the module
        """
        self.logger.info("step is started")
        self.logger.info("condition is " + self.condition.name + " @ " +
                         str(self.condition.value) +
                         "waiting for module message or time")

    def to_com_string(self):
        """
        Transform the step as a communication string for the modules
        @return the trame to be send for the target module describing each
                parameters
        """
        string = ""
        for param in self.params:
            string += "S" + param_to_frame(param.name) + chr(param.value)
        return string+"\n"

    def get_module_list(self):
        """
        Get the list of module use in the step
        @return a list of unique module
        """
        module_list = []
        for param in self.params:
            if not (param.module in module_list):
                module_list.append(param.module)
        return module_list


class StepParameter:
    """
    StepParameter, merain component of a step, a step parameter is a parameter
    to be enforced on a module.
    """

    def __init__(self, module, name = "", value = 0):
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
        return str(self.module) + " : " + str(self.name) + " @ " + \
        str(self.value)

    def is_match(self, module, name, value):
        """
        Return true if the step parameter match a given for a module
        """
        return self.module == module and self.name == name and self.value == value
