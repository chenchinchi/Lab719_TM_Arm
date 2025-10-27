# 🤖 TMrobot.py — Techman Robot ROS2 控制節點

## 📘 節點簡介

`TMrobot.py` 是一個使用 **ROS2 (rclpy)** 開發的控制節點，用於與 **Techman Robot (TM Robot)** 溝通。
此節點透過 `tm_driver` 提供的 ROS2 Service 介面，實現：

* IO 控制（開關輸出、設定腳位狀態）
* 機械手臂運動控制（位置設定、姿態移動）

---
## 🧭 範例執行流程

### 1️⃣ 啟動 TM Robot Driver

```bash
ros2 run tm_driver tm_driver robot_ip:=192.168.10.3
```

### 2️⃣ 啟動 TMrobot 節點

```bash
colcon build
source install/setup.bash
ros2 run project try_node
```

### 3️⃣ 終端互動範例

執行後會依序進行：

```
✅ tm_robot_node initialized
Enter IO state (0=OFF, 1=ON): 1
⚙️ [SetIO] module=1, type=1, pin=0, state=1
✅ SetIO response: ok=True
Press Enter to move robot to specified position...
⚙️ [SetPositions] type=LINE_T, pos=[-0.09856, 0.2995, 0.41684, 2.983, -0.036, -3.075], vel=1.0, acc=0.5, blend=0, fine=True
✅ SetPositions response: ok=True
✅ Robot successfully moved
```
## 🧱 節點架構

```
project/
├── project/
│   ├── TMrobot.py     <-- 主程式
│   └── __init__.py
├── package.xml
├── setup.py           <-- 註冊節點 entry point
└── setup.cfg
```

### setup.py 設定範例

確保 `setup.py` 的 `entry_points` 已包含：

```python
entry_points={
    'console_scripts': [
        'try_node=project.TMrobot:main',
    ],
},
```

之後即可使用以下指令執行：

```bash
ros2 run project try_node
```

---

## ⚙️ 節點功能

### 🧩 Class: `TMRobot(Node)`

主要負責與 TM Robot 進行 ROS2 Service 通訊。

| 函式名稱                                                                              | 功能說明                    |
| --------------------------------------------------------------------------------- | ----------------------- |
| `_wait_for_service(client, name)`                                                 | 等待指定 Service 可用         |
| `_deg_to_rad(deg)`                                                                | 角度轉弧度                   |
| `_mm_to_m(mm)`                                                                    | 毫米轉公尺                   |
| `IO_Set(module, io_type, pin, state)`                                             | 設定 TM Robot 的 IO 輸出     |
| `Pos_Set(motion_type, position, velocity, acc_time, blend_percentage, fine_goal)` | 發送移動指令至 TM Robot        |
| `test_io()`                                                                       | 測試 IO 控制功能（手動輸入 ON/OFF） |
| `test_move()`                                                                     | 測試位置控制功能（固定目標位置）        |

---

## 🔌 主要 Service

| Service 名稱       | 類型                         | 功能說明                 |
| ---------------- | -------------------------- | -------------------- |
| `/set_io`        | `tm_msgs/srv/SetIO`        | 控制 TM Robot 的數位輸出/輸入 |
| `/set_positions` | `tm_msgs/srv/SetPositions` | 控制 TM Robot 的關節或末端位置 |

---

## 🦾 位置控制參數說明

| 參數名稱               | 說明                                                      | 單位 / 型別     |
| ------------------ | ------------------------------------------------------- | ----------- |
| `motion_type`      | 運動模式，可選：`PTP_J`, `PTP_T`, `LINE_T`, `CIRC_T`, `PLINE_T` | `str`       |
| `position`         | 目標位置 `[X, Y, Z, Rx, Ry, Rz]`                            | `mm`, `deg` |
| `velocity`         | 運動速度                                                    | `float`     |
| `acc_time`         | 加速時間                                                    | `float`     |
| `blend_percentage` | 平滑參數 (0–100)                                            | `int`       |
| `fine_goal`        | 是否為精確終點                                                 | `bool`      |

---



---

## 🧩 常見問題 (FAQ)

| 問題                        | 原因 / 解法                           |
| ------------------------- | --------------------------------- |
| ❌ `Service not available` | 請確認 `tm_driver` 節點是否已啟動，且 IP 設定正確 |
| ❌ `SetIO failed`          | 請確認機械手臂與 PC 的連線正常，並檢查 IO 模組編號     |
| 機械手臂不動作                   | 檢查是否進入 "Remote Mode"、安全模式或有碰撞保護觸發 |

---

