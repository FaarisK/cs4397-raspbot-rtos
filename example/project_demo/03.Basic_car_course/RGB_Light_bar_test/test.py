#导入Raspbot驱动库
from Raspbot_Lib import Raspbot

bot = Raspbot()

#bot.Ctrl_WQ2812_ALL(0,1)
bot.Ctrl_WQ2812_Alone(1, 1, 0)
