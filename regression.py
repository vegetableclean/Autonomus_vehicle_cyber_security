import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.ensemble import RandomForestRegressor
from sklearn.metrics import mean_absolute_error

def load_and_process_data(folder_path, label):
    """ 讀取資料夾內的所有 CSV，並取每個 Step 最小的 Lidar_Range，並加上 Label """
    all_data = []
    
    for file_name in os.listdir(folder_path):
        if file_name.endswith(".csv"):
            file_path = os.path.join(folder_path, file_name)
            
            df = pd.read_csv(file_path, delimiter=',', encoding='utf-8-sig', skipinitialspace=True)

            # 確保欄位名稱正確
            df.columns = df.columns[0].split(',') if len(df.columns) == 1 else df.columns

            # 確保 Step 是數字
            df["Step"] = df["Step"].astype(int)

            # 取每個 Step 最小的 Lidar_Range
            df_min = df.loc[df.groupby("Step")["Lidar_Range"].idxmin()]
            df_min["Label"] = label  # 加上 malicious / normal 標籤
            
            all_data.append(df_min)

    return pd.concat(all_data, ignore_index=True) if all_data else pd.DataFrame()

def train_regression_model(train_data):
    """ 訓練隨機森林回歸模型 """
    features = ["Step", "Lidar_Bearing"]
    target = "Lidar_Range"

    X_train = train_data[features]
    y_train = train_data[target]

    model = RandomForestRegressor(n_estimators=100, random_state=42)
    model.fit(X_train, y_train)

    return model

def evaluate_model(model, test_data):
    """ 用測試集來評估模型，計算 MAE，並繪製預測 vs 真實曲線 """
    features = ["Step", "Lidar_Bearing"]
    target = "Lidar_Range"

    X_test = test_data[features]
    y_test = test_data[target]

    y_pred = model.predict(X_test)
    error = mean_absolute_error(y_test, y_pred)
    print(f"Mean Absolute Error on test data: {error:.4f}")

    test_data["Predicted_Range"] = y_pred

    # 取每個 Step 最小的 Predicted_Range
    test_data_min_pred = test_data.loc[test_data.groupby("Step")["Predicted_Range"].idxmin()]
    
    # 繪製 Malicious 數據
    plt.figure(figsize=(10, 6))
    malicious_data = test_data[test_data["Label"] == "malicious"]
    malicious_pred = test_data_min_pred[test_data_min_pred["Label"] == "malicious"]
    
    plt.scatter(malicious_data["Step"], malicious_data["Lidar_Range"], label="Actual Malicious", alpha=0.5)
    plt.plot(malicious_pred["Step"], malicious_pred["Predicted_Range"], linestyle='--', label="Predicted Malicious", color='red')

    plt.xlabel("Step")
    plt.ylabel("Lidar Range")
    plt.title("Malicious Data Prediction")
    plt.legend()
    plt.grid()
    plt.show()
    
    # 繪製 Normal 數據
    plt.figure(figsize=(10, 6))
    normal_data = test_data[test_data["Label"] == "normal"]
    normal_pred = test_data_min_pred[test_data_min_pred["Label"] == "normal"]
    
    plt.scatter(normal_data["Step"], normal_data["Lidar_Range"], label="Actual Normal", alpha=0.5)
    plt.plot(normal_pred["Step"], normal_pred["Predicted_Range"], linestyle='--', label="Predicted Normal", color='blue')

    plt.xlabel("Step")
    plt.ylabel("Lidar Range")
    plt.title("Normal Data Prediction")
    plt.legend()
    plt.grid()
    plt.show()


if __name__ == "__main__":
    # 訓練資料夾
    train_malicious_folder = r"C:\Users\User\Desktop\ACL\Lidar_data\lidar_malicious\train"
    train_normal_folder = r"C:\Users\User\Desktop\ACL\Lidar_data\lidar_normal\train"

    # 測試資料夾
    test_malicious_folder = r"C:\Users\User\Desktop\ACL\Lidar_data\lidar_malicious\test"
    test_normal_folder = r"C:\Users\User\Desktop\ACL\Lidar_data\lidar_normal\test"

    # 讀取訓練資料
    train_malicious_data = load_and_process_data(train_malicious_folder, label="malicious")
    train_normal_data = load_and_process_data(train_normal_folder, label="normal")
    train_data = pd.concat([train_malicious_data, train_normal_data], ignore_index=True)

    # 讀取測試資料
    test_malicious_data = load_and_process_data(test_malicious_folder, label="malicious")
    test_normal_data = load_and_process_data(test_normal_folder, label="normal")
    test_data = pd.concat([test_malicious_data, test_normal_data], ignore_index=True)

    # 訓練回歸模型
    model = train_regression_model(train_data)

    # 評估模型並繪圖
    evaluate_model(model, test_data)
