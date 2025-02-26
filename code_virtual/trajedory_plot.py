import csv
import math
import matplotlib.pyplot as plt

def plot_obstacle_trajectory(csv_filename):
    car_xs = []
    car_ys = []
    obs_xs = []
    obs_ys = []
    steps = []
    min_ranges = []
    
    step_data = {}  # 用來存每個 step 最小的 Lidar_Range
    
    with open(csv_filename, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            step = int(row["Step"])  # 假設 CSV 裡有 Step 欄位
            car_x = float(row["Car_X"])
            car_y = float(row["Car_Y"])
            lidar_range = float(row["Lidar_Range"])
            lidar_bearing = float(row["Lidar_Bearing"])
            
            if step not in step_data or lidar_range < step_data[step]["Lidar_Range"]:
                step_data[step] = {
                    "Car_X": car_x,
                    "Car_Y": car_y,
                    "Lidar_Range": lidar_range,
                    "Lidar_Bearing": lidar_bearing
                }
    
    # 轉換座標並存入清單
    for step, data in step_data.items():
        car_x = data["Car_X"]
        car_y = data["Car_Y"]
        lidar_range = data["Lidar_Range"]
        lidar_bearing = data["Lidar_Bearing"]
        
        obs_x = car_x + lidar_range * math.cos(lidar_bearing)
        obs_y = car_y + lidar_range * math.sin(lidar_bearing)
        
        car_xs.append(car_x)
        car_ys.append(car_y)
        obs_xs.append(obs_x)
        obs_ys.append(obs_y)
        steps.append(step)
        min_ranges.append(lidar_range)
    
    # 繪製障礙物軌跡圖
    plt.figure(figsize=(8, 8))
    plt.scatter(obs_xs, obs_ys, c='blue', label='Obstacle Trajectory')
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Obstacle and Car Trajectories")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()
    
    # 繪製最小距離隨 step 變化圖
    plt.figure(figsize=(8, 6))
    plt.plot(steps, min_ranges, marker='o', linestyle='-', color='red', label='Min Lidar Range per Step')
    plt.xlabel("Step")
    plt.ylabel("Min Lidar Range")
    plt.title("Minimum Lidar Range per Step")
    plt.legend()
    plt.grid(True)
    plt.show()



if __name__ == "__main__":
    # 依你的 CSV 檔案路徑做修改
    csv_file = r"C:\Users\User\Desktop\ACL\Lidar_data\lidar_malicious\lidar_data7.csv"
    plot_obstacle_trajectory(csv_file)