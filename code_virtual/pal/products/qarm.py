import numpy as np
from quanser.hardware import HIL, HILError, MAX_STRING_LENGTH, Clock
from quanser.hardware.enumerations import BufferOverflowMode
from pal.utilities.vision import Camera3D


class QArm():

    '''QArm class for initialization, I/O and termination'''
    #region: Channel and Buffer definitions

    # Channels
    WRITE_OTHER_CHANNELS = np.array([1000, 1001, 1002, 1003, 1004, 11000, 11001, 11002, 11003, 11005, 11006, 11007], dtype=np.int32)
    READ_OTHER_CHANNELS = np.array([1000, 1001, 1002, 1003, 1004, 3000, 3001, 3002, 3003, 3004, 10000, 10001, 10002, 10003, 10004, 11000, 11001, 11002, 11003, 11004], dtype=np.int32)
    READ_ANALOG_CHANNELS = np.array([5, 6, 7, 8, 9], dtype=np.int32)

    # Buffers (internal)
    writeOtherBuffer = np.zeros(12, dtype=np.float64)
    readOtherBuffer = np.zeros(20, dtype=np.float64)
    readAnalogBuffer = np.zeros(5, dtype=np.float64)

    # Buffers (external)
    measJointCurrent        = np.zeros(5, dtype=np.float64)
    measJointPosition       = np.zeros(5, dtype=np.float64)
    measJointSpeed          = np.zeros(5, dtype=np.float64)
    measJointPWM            = np.zeros(5, dtype=np.float64)
    measJointTemperature    = np.zeros(5, dtype=np.float64)

    status = False
    #endregion

    def __init__(self, hardware=1, readMode=0, frequency=500, deviceId = 0, hilPortNumber = 18900):
        ''' This function configures the QArm in Position (0) or PWM (1) mode
        based on the input, and returns a handle to the card. Use the handle
        for other read/write methods. .'''
        self.readMode = readMode
        self.hardware = hardware
        self.mode = np.array([0, 0, 0, 0, 0], dtype=np.uint8) # only supporting
                                                              # position mode
                                                              # at the moment
        self.card = HIL()
        if self.hardware:
            boardIdentifier = str(deviceId)
        else:
            boardIdentifier = "0@tcpip://localhost:" + str(hilPortNumber) + "?nagle='off'"

        boardSpecificOptions = f"j0_mode={int(self.mode[0])};j1_mode={int(self.mode[1])};j2_mode={int(self.mode[2])};j3_mode={int(self.mode[3])};gripper_mode={int(self.mode[4])};j0_profile_config=0;j0_profile_velocity=1.5708;j0_profile_acceleration=1.0472;j1_profile_config=0;j1_profile_velocity=1.5708;j1_profile_acceleration=1.0472;j2_profile_config=0;j2_profile_velocity=1.5708;j2_profile_acceleration=1.0472;j3_profile_config=0;j3_profile_velocity=1.5708;j3_profile_acceleration=1.0472;"

        try:
            # Open the Card
            self.card.open("qarm_usb", boardIdentifier)
            if self.card.is_valid():
                self.card.set_card_specific_options(boardSpecificOptions, MAX_STRING_LENGTH)
                self.status = True
                if self.readMode == 1:
                    self.frequency = frequency
                    # Define reading task
                    self.readTask = self.card.task_create_reader(   int(self.frequency*2),
                                                                    self.READ_ANALOG_CHANNELS, len(self.READ_ANALOG_CHANNELS),
                                                                    None, 0,
                                                                    None, 0,
                                                                    self.READ_OTHER_CHANNELS, len(self.READ_OTHER_CHANNELS))

                    # Set buffer overflow mode depending on whether its for hardware or virtual QCar
                    if self.hardware:
                        self.card.task_set_buffer_overflow_mode(self.readTask, BufferOverflowMode.OVERWRITE_ON_OVERFLOW)
                        self.card.set_double_property(np.array([128 , 129, 130, 131,   133,   134,   135,   136,  138,  139,  140,  141]),
                                                      12,
                                                      np.array([8.89,8.89,8.89,8.89, 0.012, 0.012, 0.012, 0.012,10.23,10.23,10.23,10.23]))
                        print('setting PID gains')
                    else:
                        self.card.task_set_buffer_overflow_mode(self.readTask, BufferOverflowMode.WAIT_ON_OVERFLOW)

                    # Start the reading task
                    self.card.task_start(self.readTask, Clock.HARDWARE_CLOCK_0, self.frequency, 2**32-1)


                else:
                    if self.hardware:
                        self.card.set_double_property(np.array([128 , 129, 130, 131,   133,   134,   135,   136,  138,  139,  140,  141]),
                                                      12,
                                                      np.array([8.89,8.89,8.89,8.89, 0.012, 0.012, 0.012, 0.012,10.23,10.23,10.23,10.23]))
                        print('setting PID gains')
                    print('QArm configured in Position Mode.')

        except HILError as h:
            print(h.get_error_message())

    def terminate(self):
        ''' This function terminates the QArm card after setting final values for home position and 0 pwm.'''

        try:
            self.card.close()
            print('QArm terminated successfully.')

        except HILError as h:
            print(h.get_error_message())

    def read_write_std(self, phiCMD=np.zeros(4, dtype=np.float64),
                        grpCMD=np.zeros(1, dtype=np.float64),
                        baseLED=np.array([1, 0, 0], dtype=np.float64)):
        '''Use this to write motor and LED commands, and read the battery voltage, motor current and encoder counts \n

        INPUTS:
        phiCMD - angular position of joints 1 to 4 as a 4x1 numpy array (active in Position mode only)
        grpCMD - gripper position 1x1 numpy array
        baseLED - base RGB LED state as a 3x1 numpy array

        OUTPUTS:
        None, use the self variables - measJointCurrent, measJointPosition, measJointSpeed, measJointPWM and measJointTemperature'''
        self.writeOtherBuffer[4] = np.clip(grpCMD,0.1,0.9) # Saturate gripper between 0.1 (open) and 0.9 (close)
        self.writeOtherBuffer[9:12] = baseLED
        new=False
        for motorIndex in range(4):
            if self.mode[motorIndex]:
                self.writeOtherBuffer[5+motorIndex] = np.zeros(1, dtype=np.float64)
            else:
                self.writeOtherBuffer[motorIndex] = phiCMD[motorIndex]

        # IO
        try:
            if True:
                #Writes: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer
                self.card.write(None, 0,
                                None, 0,
                                None, 0,
                                self.WRITE_OTHER_CHANNELS, 12,
                                None,
                                None,
                                None,
                                self.writeOtherBuffer)

                #Reads: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer
                self.card.read( self.READ_ANALOG_CHANNELS, 5,
                                None, 0,
                                None, 0,
                                self.READ_OTHER_CHANNELS, 20,
                                self.readAnalogBuffer,
                                None,
                                None,
                                self.readOtherBuffer)
                new=True

        except HILError as h:
            print(h.get_error_message())
            new = False

        finally:
            self.measJointCurrent       = self.readAnalogBuffer
            self.measJointPosition      = self.readOtherBuffer[0:5]
            self.measJointSpeed         = self.readOtherBuffer[5:10]
            self.measJointPWM           = self.readOtherBuffer[15:20]
            self.measJointTemperature   = self.readOtherBuffer[10:15]
            return new

    def read_std(self):
        '''Use this to read the battery voltage, motor current and encoder counts \n

        OUTPUTS:
        Use the class variables - measJointCurrent, measJointPosition, measJointSpeed, measJointPWM and measJointTemperature'''

        # IO
        new=False
        try:
            if True:
                #Reads: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer
                self.card.read( self.READ_ANALOG_CHANNELS, 5,
                                None, 0,
                                None, 0,
                                self.READ_OTHER_CHANNELS, 20,
                                self.readAnalogBuffer,
                                None,
                                None,
                                self.readOtherBuffer)
                new=True
        except HILError as h:
            print(h.get_error_message())
            new=False
        finally:
            self.measJointCurrent = self.readAnalogBuffer
            self.measJointPosition = self.readOtherBuffer[0:5]
            self.measJointSpeed = self.readOtherBuffer[5:10]
            self.measJointPWM = self.readOtherBuffer[15:20]
            self.measJointTemperature = self.readOtherBuffer[10:15]
            return new
    def write_LEDs(self, baseLED=np.array([1, 0, 0], dtype=np.float64)):
        '''Use this to write LED commands (eg. to verify functionality HIL Card access) \n

        INPUTS:
        baseLED - base RGB LED state as a 3x1 numpy array '''
        self.writeOtherBuffer[9:] = baseLED

        # IO
        try:
            if True:
                #Writes: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer
                self.card.write(None, 0,
                                None, 0,
                                None, 0,
                                self.WRITE_OTHER_CHANNELS[9:], 3,
                                None,
                                None,
                                None,
                                self.writeOtherBuffer[9:])

        except HILError as h:
            print(h.get_error_message())

        
class QArmRealSense(Camera3D):
    """
    A class for accessing 3D camera data from the RealSense camera on the QBot
    Platform.

    Inherits from Camera3D class in pal.utilities.vision

    Args:
        mode (str): Mode to use for capturing data. Default is 'RGB&DEPTH'.
        frameWidthRGB (int): Width of the RGB frame. Default is 640.
        frameHeightRGB (int): Height of the RGB frame. Default is 400.
        frameRateRGB (int): Frame rate of the RGB camera. Default is 30.
        frameWidthDepth (int): Width of the depth frame. Default is 640.
        frameHeightDepth (int): Height of the depth frame. Default is 400.
        frameRateDepth (int): Frame rate of the depth camera. Default is 15.
        frameWidthIR (int): Width of the infrared (IR) frame. Default is 640.
        frameHeightIR (int): The height of the IR frame. Default is 400.
        frameRateIR (int): Frame rate of the IR camera. Default is 15.
        readMode (int): Mode to use for reading data from the camera.
            Default is 1.
        focalLengthRGB (numpy.ndarray): RGB camera focal length in pixels.
            Default is np.array([[None], [None]], dtype=np.float64).
        principlePointRGB (numpy.ndarray): Principle point of the RGB camera
            in pixels. Default is np.array([[None], [None]], dtype=np.float64).
        skewRGB (float): Skew factor for the RGB camera. Default is None.
        positionRGB (numpy.ndarray): An array of shape (3, 1) that holds the
            position of the RGB camera in the QBot's frame of reference.
        orientationRGB (numpy.ndarray): An array of shape (3, 3) that holds the
            orientation of the RGB camera in the QBot's frame of reference.
        focalLengthDepth (numpy.ndarray): An array of shape (2, 1) that holds
            the focal length of the depth camera.
        principlePointDepth (numpy.ndarray): An array of shape (2, 1) that
            holds the principle point of the depth camera.
        skewDepth (float, optional): Skew of the depth camera
        positionDepth (numpy.ndarray, optional): An array of shape (3, 1) that
            holds the position of the depth camera
        orientationDepth (numpy.ndarray): An array of shape (3, 3) that holds
            the orientation of the Depth camera in the QBot's reference frame.
    """
    def __init__(
            self,
            hardware = 1,
            videoPortNumber = 18901,
            mode='RGB&DEPTH',
            deviceID = 0,
            readMode=1,
            frameWidthRGB=640,
            frameHeightRGB=400,
            frameRateRGB=30,
            frameWidthDepth=640,
            frameHeightDepth=400,
            frameRateDepth=15,
            frameWidthIR=640,
            frameHeightIR=400,
            frameRateIR=15,
            focalLengthRGB=np.array([[None], [None]], dtype=np.float64),
            principlePointRGB=np.array([[None], [None]], dtype=np.float64),
            skewRGB=None,
            positionRGB=np.array([[None], [None], [None]], dtype=np.float64),
            orientationRGB=np.array([[None, None, None], [None, None, None],
                                     [None, None, None]], dtype=np.float64),
            focalLengthDepth=np.array([[None], [None]], dtype=np.float64),
            principlePointDepth=np.array([[None], [None]], dtype=np.float64),
            skewDepth=None,
            positionDepth=np.array([[None], [None], [None]], dtype=np.float64),
            orientationDepth=np.array([[None, None, None], [None, None, None],
                                       [None, None, None]], dtype=np.float64)
        ):

        if hardware:
            deviceId = str(deviceID)
        else:
            deviceId = "0@tcpip://localhost:" + str(videoPortNumber)
            frameWidthRGB = 640
            frameHeightRGB = 480
            frameRateRGB = 30
            frameWidthDepth = 640
            frameHeightDepth = 480
            frameRateDepth = 15
            frameWidthIR = 640
            frameHeightIR = 480
            frameRateIR = 30

        super().__init__(
            mode,
            frameWidthRGB,
            frameHeightRGB,
            frameRateRGB,
            frameWidthDepth,
            frameHeightDepth,
            frameRateDepth,
            frameWidthIR,
            frameHeightIR,
            frameRateIR,
            deviceId,
            readMode,
            focalLengthRGB,
            principlePointRGB,
            skewRGB,
            positionRGB,
            orientationRGB,
            focalLengthDepth,
            principlePointDepth,
            skewDepth,
            positionDepth,
            orientationDepth
        )