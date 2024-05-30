# 灯光控制使能，控制必须使能灯光控制
uint8 CONTROL_DISABLE = 0
uint8 CONTROL_ENABLE = 1

# 灯光控制模式
# 0x00: 常关
# 0x01: 常开(蓝色)
# 0x02: 黄色呼吸灯闪烁
# 0x03: 开放客户权限
# 0x04：急停红色
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
