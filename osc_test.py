from Raspbot_Lib import Raspbot
import socket
import struct
import math


class BotRef:
    def __init__(self, bot):
        self.bot = bot

    def __del__(self):
        self.bot.Ctrl_Muto(0, 0)
        self.bot.Ctrl_Muto(1, 0)
        self.bot.Ctrl_Muto(2, 0)
        self.bot.Ctrl_Muto(3, 0)
        self.bot.Ctrl_Servo(1, 90)
        self.bot.Ctrl_Servo(2, 25)
        print("Exiting!!!")


bot_ref = BotRef(Raspbot())

ROOT2 = math.sqrt(2)
WHEEL_PHASE = math.pi / 4
MAX_SPEED = 100


def ctrl_grid(x, y):
    move_angle = math.atan2(y, x)
    wheel_cos = math.cos(move_angle - WHEEL_PHASE)
    wheel_sin = math.sin(move_angle - WHEEL_PHASE)
    speed = math.sqrt(x * x + y * y) / ROOT2

    fl = wheel_cos * speed * MAX_SPEED
    rl = wheel_sin * speed * MAX_SPEED
    fr = wheel_sin * speed * MAX_SPEED
    rr = wheel_cos * speed * MAX_SPEED
    print(format(fl, '.3'), format(fr, '.3'))
    print(format(rl, '.3'), format(rr, '.3'))
    print(format(speed, '.3'))
    print()

    bot_ref.bot.Ctrl_Muto(0, round(fl))
    bot_ref.bot.Ctrl_Muto(1, round(rl))
    bot_ref.bot.Ctrl_Muto(2, round(fr))
    bot_ref.bot.Ctrl_Muto(3, round(rr))


def ctrl_radial(r, a):
    a -= 0.5

    print(r, a)


def read_int(buf, i):
    return 4, struct.unpack('>i', memoryview(buf)[i:i + 4])[0]


def read_float(buf, i):
    return 4, struct.unpack('>f', memoryview(buf)[i:i + 4])[0]


buffer = bytearray(8192)
bufview = memoryview(buffer)

convert = dict()
convert['f'] = read_float
convert['i'] = read_int

methods = dict()
methods['/ctrl/grid'] = ctrl_grid
methods['/ctrl/radial'] = ctrl_radial

sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
sock.bind(('', 8254))
while True:
    read = sock.recv_into(bufview)
    # print(buffer[:read].hex())

    addr_end = buffer.find(0)
    addr = buffer[0:addr_end].decode('ascii')

    print(addr)
    method = methods.get(addr)
    if method is None:
        continue

    if addr_end & 3:
        addr_end = (addr_end & ~3) + 4
    else:
        addr_end += 4

    fmt_end = buffer.find(0, addr_end)
    fmt = buffer[addr_end:fmt_end].decode('ascii')

    if fmt_end & 3:
        fmt_end = (fmt_end & ~3) + 4
    else:
        fmt_end += 4

    args = []
    i = fmt_end
    for c in fmt[1:]:
        read, arg = convert[c](buffer, i)
        args.append(arg)
        i += read

    # print(addr_end, fmt_end, addr, fmt, args)
    # print(args)
    method(*args)
