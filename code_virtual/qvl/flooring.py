from qvl.qlabs import CommModularContainer
from qvl.actor import QLabsActor

import numpy as np
import math
import struct


######################### MODULAR CONTAINER CLASS #########################

class QLabsFlooring(QLabsActor):
    """ This is a deprecated class.  Please see the product-specific flooring classes."""

    ID_FLOORING = 10090
    """Class ID"""

    FLOORING_QCAR_MAP_LARGE = 0
    FLOORING_QCAR_MAP_SMALL = 1


    def __init__(self, qlabs, verbose=False):
        """ Constructor Method

        :param qlabs: A QuanserInteractiveLabs object
        :param verbose: (Optional) Print error information to the console.
        :type qlabs: object
        :type verbose: boolean
        """
        print('The class QLabsFlooring will be deprecated starting 2025. Please use QLabsQCarFlooring or QLabsQBotPlatformFlooring.')
        self._qlabs = qlabs
        self._qlabs = qlabs
        self._verbose = verbose
        self.classID = self.ID_FLOORING
        return