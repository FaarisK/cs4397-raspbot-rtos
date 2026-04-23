from Raspbot_Lib import Raspbot
color = Raspbot()


# 颜色到灯珠颜色编号的映射
color_to_number = {
        'red': 0,
        'green': 1,
        'blue': 2,
        'yellow': 3
    }
def light_2leds(color1,color2):
    # 获取颜色对应的灯珠颜色编号
    color1_number = color_to_number.get(color1, None)
    color2_number = color_to_number.get(color2, None)

    # 分配灯珠
    for i in range(1, 6):  # 前 5 个灯珠
        color.Ctrl_WQ2812_Alone(i, 1, color1_number)
    for i in range(6, 11):  # 后 5 个灯珠
        color.Ctrl_WQ2812_Alone(i, 1, color2_number)

def light_3leds(color1,color2,color3):
    # 获取颜色对应的灯珠颜色编号
    color1_number = color_to_number.get(color1, None)
    color2_number = color_to_number.get(color2, None)
    color3_number = color_to_number.get(color3, None)

    # 分配灯珠
    for i in range(1, 4):  # 前 3 个灯珠
        color.Ctrl_WQ2812_Alone(i, 1, color1_number)
    for i in range(4, 8):  # 中 4 个灯珠
        color.Ctrl_WQ2812_Alone(i, 1, color2_number)
    for i in range(8, 11):  # 后 3 个灯珠
        color.Ctrl_WQ2812_Alone(i, 1, color3_number)

def light_4leds(color1,color2,color3,color4):
    # 获取颜色对应的灯珠颜色编号
    color1_number = color_to_number.get(color1, None)
    color2_number = color_to_number.get(color2, None)
    color3_number = color_to_number.get(color3, None)
    color4_number = color_to_number.get(color4, None)

    # 分配灯珠
    for i in range(1, 4):  # 前 3 个灯珠
        color.Ctrl_WQ2812_Alone(i, 1, color1_number)
    for i in range(4, 6):  # 后 2 个灯珠
        color.Ctrl_WQ2812_Alone(i, 1, color2_number)
    for i in range(6, 8):  # 后 2 个灯珠
        color.Ctrl_WQ2812_Alone(i, 1, color3_number)
    for i in range(9, 11):  # 后 3 个灯珠
        color.Ctrl_WQ2812_Alone(i, 1, color4_number)
    
def light_leds(unique_colors):
    if(len(unique_colors)==0):
        color.Ctrl_WQ2812_ALL(0,0)
    elif(len(unique_colors)==1):
        color.Ctrl_WQ2812_ALL(1,color_to_number.get(*unique_colors, None))
    elif(len(unique_colors)==2):
        light_2leds(*unique_colors)
    elif(len(unique_colors)==3):   
        light_3leds(*unique_colors)
    elif(len(unique_colors)==4):   
        light_4leds(*unique_colors)