# region: package imports
import os
import numpy as np
from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar import QLabsQCar
from qvl.free_camera import QLabsFreeCamera
from qvl.real_time import QLabsRealTime
from qvl.traffic_cone import QLabsTrafficCone
import pal.resources.rtmodels as rtmodels
import time

# region: setup environment with only traffic cones
def setup(
    initialPosition=[2.0, 20.0, 0.0],
    initialOrientation=[-1, 0, -np.pi/2],
    rtModel=rtmodels.QCAR
):
    os.system('cls')
    qlabs = QuanserInteractiveLabs()
    print("Connecting to QLabs...")
    try:
        qlabs.open("localhost")
        print("Connected to QLabs")
    except:
        print("Unable to connect to QLabs")
        quit()

    # Clear previous environment and objects
    qlabs.destroy_all_spawned_actors()
    QLabsRealTime().terminate_all_real_time_models()

    # region: Initialize QCar
    hqcar = QLabsQCar(qlabs)
    hqcar.spawn_id(
        actorNumber=0,
        location=[x for x in initialPosition],
        rotation=initialOrientation,
        waitForConfirmation=True
    )

    hcamera = QLabsFreeCamera(qlabs)
    hcamera.spawn([8.484, 1.973, 12.209], [-0, 0.748, 0.792])
    hqcar.possess()
    # endregion

    # region: Spawn only traffic cones
    hTrafficCone = QLabsTrafficCone(qlabs)  # 新增交通錐物件
    hTrafficCone.spawn_id(
        actorNumber=101,  # 設定交通錐的編號
        location=[2.0, 3.0, 0.0],  # 移動交通錐位置到車子前方
        rotation=[0, 0, np.pi/2],  # 旋轉交通錐以便正確顯示
        scale=[7, 7, 7],  # 設定大小
        configuration=0,  # 配置設置
        waitForConfirmation=True  # 等待確認
    )

    # 設定交通錐顏色
    hTrafficCone.set_material_properties(
        materialSlot=0, color=[0, 0, 1], metallic=False  # 這是橙色
    )
    time.sleep(0.5)  # 等待顯示效果
    # endregion

    QLabsRealTime().start_real_time_model(rtModel)
    return hqcar


if __name__ == "__main__":
    setup()
