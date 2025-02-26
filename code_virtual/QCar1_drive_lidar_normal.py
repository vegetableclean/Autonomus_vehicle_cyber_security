import os
import numpy as np
import time
import csv
import matplotlib.pyplot as plt
from qvl.qlabs import QuanserInteractiveLabs
#from qvl.qcar import QLabsQCar
from pal.products.qcar import QCar
from qvl.traffic_cone import QLabsTrafficCone
import pal.resources.rtmodels as rtmodels
from pal.products.qcar import QCarLidar
from qvl.real_time import QLabsRealTime

# 收集 LIDAR 資料並存成 CSV 檔
def drive_and_collect(qcar, lidar, filename='Lidar_data.csv', max_steps=200):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Step", "Car_X", "Car_Y", "Lidar_Range", "Lidar_Bearing"])
        step = 0
        while step < max_steps:
            lidar.read()
            distances = np.array(lidar.distances)
            angles = np.array(lidar.angles)
            
            # 過濾出符合條件 (0 < 距離 < 4) 的數據
            valid_mask = (distances > 0) & (distances < 4)
            if np.any(valid_mask):
                valid_distances = distances[valid_mask]
                valid_angles = angles[valid_mask]
                # 找到最小距離的索引
                min_index = np.argmin(valid_distances)
                min_distance = valid_distances[min_index]
                min_angle = valid_angles[min_index]
                x = 0            # 假設車輛一直往前移動，x 為常數或依需求計算
                y = step * 0.1   # 假設車輛以固定速度前進，y 隨著 step 增加
                writer.writerow([step, x, y, min_distance, min_angle])
                # 判斷最小距離，若小於 0.5 則右轉，否則直走
                if min_distance < 1:
                    # 右轉：這裡假設 steering=0.3 代表右轉，依實際情況調整數值
                    qcar.read_write_std(throttle=0.1, steering=0.5)
                else:
                    qcar.read_write_std(throttle=0.1, steering=0.0)
            else:
                qcar.read_write_std(throttle=0.1, steering=0.0)  # 慢速前進
            time.sleep(0.1)
            step += 1
    qcar.terminate()

# 讀取 CSV 並以散點圖方式繪製每個 step 的 distance
def plot_data(filename='Lidar_data.csv'):
    steps = []
    distances = []
    with open(filename, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            step = int(row["Step"])
            distance = float(row["Lidar_Range"])
            steps.append(step)
            distances.append(distance)
    
    plt.figure(figsize=(10,6))
    plt.scatter(steps, distances, alpha=0.6, marker='o')
    plt.xlabel("Step")
    plt.ylabel("Distance")
    plt.title("Distance for Each Step (Scatter Plot)")
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    qlabs = QuanserInteractiveLabs()
    qlabs.open("localhost")
    qcar = QCar(qlabs)
    lidar = QCarLidar(numMeasurements=360)
    
    # 收集資料 (此處設定 max_steps 為 100，可依需求修改)
    drive_and_collect(qcar, lidar, max_steps=200)
    
    # 繪製資料圖形
    plot_data()
