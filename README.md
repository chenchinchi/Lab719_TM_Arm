# Lab719_TM_robot_arm_control

## 環境配置
```
ubuntu 22.04
ROS2 humble
TM 1.8
```

## 下載TM的函式庫
```
mkdir tm_ws
cd tm_ws
git clone https://github.com/TechmanRobotInc/tmr_ros2.git -b humble
mv tmr_ros2 src
```

## 編譯
當在src的程式中內容有改動的時候需要重新編譯
```
colcon build #編譯
source install/setup.bash #載入
```

## 新增程式
若有新增python 節點需要改動setup.py中的entry_points
```python
from setuptools import find_packages, setup

package_name = 'edcra'

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
    maintainer='abc',
    maintainer_email='abc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ # 新增在這邊 '<要用的程式代號>=<資料夾名稱>.<檔案>:main'
        	'sensor=edcra.test_sensor:main',
        	'image=edcra.test_image:main',
        	'ui=edcra.ui_window:main',
        ],
    },
)
```






## 測試  
1. 先開啟終端機和手臂連線
```
source install/setup.bash
ros2 run tm_driver tm_driver robot_ip:=192.168.10.3
```
2. 開啟第二個終端機
```
source install/setup.bash
ros2 run demo <demo/src資料夾中的程式>
```



