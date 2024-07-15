# ROS Packages for Iray Tracer Mobile Base

## ROS包结构

* tracer_base: a ROS wrapper around tracer SDK to monitor and control the robot
* tracer_bringup: launch and configuration files to start ROS nodes
* tracer_msgs: tracer related message definitions

## 使用方法

1. Clone the packages into your catkin workspace and compile

   (the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

   ```
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/agilexrobotics/iray_ros.git
   $ cd ..
   $ catkin_make
   ```

2. Setup CAN-To-USB adapter

* Enable gs_usb kernel module(If you have already added this module, you do not need to add it)

  ```
  $ sudo modprobe gs_usb
  ```

* first time use tracer-ros package

  ```
  $rosrun tracer_bringup setup_can2usb.bash
  ```

* If not the first time use tracer-ros package(Run this command every time you turn on the power)

  ```
  $rosrun tracer_bringup bringup_can2usb.bash
  ```

4. Launch ROS nodes

* Start the base node for the real robot whith can

  ```
  $ roslaunch tracer_bringup tracer_robot_base.launch
  ```

* Start the keyboard tele-op node

  ```
  $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
  ```

## ROS话题反馈和功能

### 车辆信息/tracer_status

```bash
# 时间戳
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id

# 运动状态反馈
float64 linear_velocity
float64 angular_velocity

# 车辆状态反馈
uint8 NORMAL = 0            #车辆正常
uint8 EMERGENCY_STOP = 1    #急停
uint8 SYSTEM_FAULT = 2      #系统异常
uint8 base_state

uint8 STANDBY = 0           #待机模式
uint8 CAN_MODE = 1          #CAN指令控制模式
uint8 UART_MODE = 2         #串口控制模式
uint8 REMOTE_MOED = 3       #遥控模式
uint8 control_mode

# 错误码，详情见码表
uint8 fault_code

# 电池电压
float64 battery_voltage

# 电机状态反馈
TracerMotorState[2] motor_states

# light state
# bool light_control_enabled
# TracerLightState front_light_state

#odometer state
float64 left_odomter
float64 right_odomter
```
### 电池信息反馈/battery_state

```bash
std_msgs/Header header //时间戳
  uint32 seq
  time stamp
  string frame_id
uint32 seq
time stamp
string frame_id
float32 voltage //电池电压
float32 temperature //电池温度
float32 current //电池电流
float32 percentage //电量百分比
bool is_charge // 当前是否为充电状态
```

### 重置里程计话题/reset_odom

``` bash
rostopic pub /reset_odom tracer_msgs/ResetOdom "clear: {}"
```

发送该话题消息，里程计累计历程清零

### 清除所有错误码话题/clear_err

``` bash
rostopic pub /clear_err tracer_msgs/ClearErr "clear: {}"
```

发送该话题消息，清除车辆所有错误码和异常状态

### 灯光控制话题/tracer_light_control

```bash
# enable_cmd_light_control
# 灯光控制使能，控制必须使能灯光控制
uint8 CONTROL_DISABLE = 0
uint8 CONTROL_ENABLE = 1

# mode 
# 灯光控制模式
# 0x00: 常关
# 0x01: 常开(蓝色)
# 0x02: 黄色呼吸灯闪烁
# 0x03: 开放客户权限
# 0x04：急停红色

# RGB value
# rgb值域范围：[0,100]，其中0为不亮，100最亮
uint8 LIGHT_OFF = 0
uint8 LIGHT_ON_BLUE = 1
uint8 LIGHT_BREATH_YELLOW = 2
uint8 LIGHT_CUSTOM = 3
uint8 LIGHT_STOP_RED = 4

uint8 enable_cmd_light_control
uint8 mode
uint8 R_value
uint8 G_value
uint8 B_value
```

## CAN test

this part is used for agilex's developers to test the CAN protocol

- enable the virtual can net

  ``` bash
  sudo modprobe vcan
  sudo ip link add dev vcan0 type vcan
  sudo ip link set up vcan0
  ```

**SAFETY PRECAUSION**: 

Always have your remote controller ready to take over the control whenever necessary. 