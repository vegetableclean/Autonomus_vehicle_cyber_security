import os


#__rtModelDirPath = os.environ['RTMODELS_DIR']
__rtModelDirPath = r'C:/Users/User\Desktop/ACL/src/examples/sdcs/'

# QCar RT Models
QCAR = os.path.normpath(
    os.path.join(__rtModelDirPath, 'qcar/QCar_Workspace'))

QCAR_STUDIO = os.path.normpath(
    os.path.join(__rtModelDirPath, 'qcar/QCar_Workspace_Studio'))

QBOT_PLATFORM = os.path.normpath(
    os.path.join(__rtModelDirPath, 'QBotPlatform/QBotPlatform_Workspace'))

QBOT_PLATFORM_DRIVER = os.path.normpath(
    os.path.join(__rtModelDirPath, 'QBotPlatform/qbot_platform_driver_virtual'))
