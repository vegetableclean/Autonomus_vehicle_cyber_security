from qvl.qlabs import CommModularContainer
from qvl.actor import QLabsActor

from quanser.common import GenericError
import math
import os
import struct


######################### MODULAR CONTAINER CLASS #########################

class QLabsQbot(QLabsActor):
    # This is a deprecated class.  Please see the product-specific qbot classes.
    
    ID_QBOT = 20

    FCN_QBOT_COMMAND_AND_REQUEST_STATE = 10
    FCN_QBOT_COMMAND_AND_REQUEST_STATE_RESPONSE = 11
    FCN_QBOT_POSSESS = 20
    FCN_QBOT_POSSESS_ACK = 21


    VIEWPOINT_RGB = 0
    VIEWPOINT_DEPTH = 1
    VIEWPOINT_TRAILING = 2

    # Initialize class
    def __init__(self, qlabs, verbose=False):
        self._qlabs = qlabs
        self._verbose = verbose
        self.classID = self.ID_QBOT
        print('The class QLabsQbot will be deprecated starting 2025. Please use product specific classes (QLabsQBot2e/QLabsQBot3/QLabsQBotPlatform).')
        
        return
   
    def possess(self, camera):
        c = CommModularContainer()
        c.classID = self.ID_QBOT
        c.actorNumber = self.actorNumber
        c.actorFunction = self.FCN_QBOT_POSSESS
        c.payload = bytearray(struct.pack(">B", camera))
        c.containerSize = c.BASE_CONTAINER_SIZE + len(c.payload)

        self._qlabs.flush_receive()
            
        if (self._qlabs.send_container(c)):
            c = self._qlabs.wait_for_container(self.ID_QBOT, self.actorNumber, self.FCN_QBOT_POSSESS_ACK)
            if (c == None):
                return False
            else:
                return True
        else:
            return False            
