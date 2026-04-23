#导入Raspbot驱动库
from Raspbot_Lib import Raspbot,LightShow
import time,sys,signal
bot = Raspbot()

lights = LightShow()


def signal_handler(sig, frame):
        print('Stopping the light show...')
        lights.stop()
        lights.turn_off_all_lights()
        sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

try:
    while True:
            lights.execute_effect('starlight',lights.MAX_TIME,0.1,-1)
except KeyboardInterrupt:
        sys.exit(0)