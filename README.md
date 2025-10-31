# 🦾 Lab719_TM_Robot_Arm_Control

## 🧩 環境配置

```
Ubuntu 22.04  
ROS2 Humble  
TM Robot 1.8
```

---

## 📥 Repo 使用方法

### 下載與建置

```bash
# 建立工作區
mkdir tm_ws
cd tm_ws

# 下載專案
git clone https://github.com/chenchinchi/Lab719_TM_Arm.git

# 移動至 ROS2 預設結構
mv Lab719_TM_Arm src

# 建置專案
colcon build

# 載入環境變數
source install/setup.bash
```

---

## 🚀 執行方法

### 1️⃣ 啟動 Driver 節點（與 TM 機器手臂通訊）

開啟終端機輸入：

```bash
source install/setup.bash
ros2 run tm_driver tm_driver robot_ip:=192.168.10.3
```

---

### 2️⃣ 啟動控制節點

開啟另一個終端機輸入：

```bash
# 若有修改程式碼，需重新編譯
colcon build

# 載入環境
source install/setup.bash

# 執行控制節點
ros2 run project try_node
# 或一般形式：
ros2 run <pkg_name> <entry_name>
```
若要使用TMrobot/TMRobot
```python
from project.TMrobot import TMRobot
robot = TMRobot()
```

---

## 📁 專案文件結構

```bash
tm_ws
└── src
    ├── project                    # 控制邏輯與使用者自訂節點
    │   ├── package.xml
    │   ├── project/               # 實際 Python 程式放在這裡
    │   ├── resource
    │   ├── setup.cfg
    │   ├── setup.py               # 新增節點時須修改此檔
    │   └── test
    │
    ├── tm_driver                  # 與 TM 機器手臂通訊的 Driver
    │   ├── CMakeLists.txt
    │   ├── include
    │   ├── launch
    │   ├── package.xml
    │   └── src
    │
    └── tm_msgs                    # 與 Driver 溝通的訊息格式定義
        ├── CMakeLists.txt
        ├── msg
        ├── package.xml
        └── srv
```

---

## ⚙️ 在 `setup.py` 新增節點方法

> 當你新增新的 Python 節點時，需在 `entry_points` 中註冊執行入口。

```python
from setuptools import find_packages, setup

package_name = 'project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lab719',
    maintainer_email='lab719@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'try_node=project.TMrobot:main',
            # ↑ 在此新增節點
            # 格式：<entry_name> = <模組路徑>:<主函式名稱>
        ],
    },
)
```

---

## 💡 小提醒

* 每次修改 Python 程式碼後請重新執行：

  ```bash
  colcon build
  source install/setup.bash
  ```
* 若無法執行，請確認 `entry_points` 是否正確綁定 `main()` 函式。
* 建議使用 `tm_driver` 與 `tm_msgs` 提供的介面與 TM 機器手臂互動。

---
