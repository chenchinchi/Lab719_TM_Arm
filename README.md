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
3. 手臂回傳OK即可


