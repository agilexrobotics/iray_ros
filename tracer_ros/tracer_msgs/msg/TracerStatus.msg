Header header

# 运动状态反馈
float64 linear_velocity
float64 angular_velocity

# 车辆状态反馈
uint8 NORMAL = 0            #车辆正常
uint8 EMERGENCY_STOP = 1    #急停
uint8 SYSTEM_FAULT = 0      #系统异常
uint8 base_state

uint8 STANDBY = 0           #待机模式
uint8 CAN_MODE = 1          #CAN指令控制模式
uint8 UART_MODE = 2         #串口控制模式
uint8 REMOTE_MOED = 3       #遥控模式
uint8 control_mode

# 错误码，详情见码表
uint8 fault_code

#电池电压
float64 battery_voltage

# 电机状态反馈
TracerMotorState[2] motor_states

# light state
# bool light_control_enabled
# TracerLightState front_light_state

#odometer state
float64 left_odomter
float64 right_odomter
