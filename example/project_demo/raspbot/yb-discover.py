import os
import sys
import getopt
import socket
#import time


if __name__ == "__main__":
    host = '0.0.0.0'
    port = 8000
    addr = (host, port)
    udpServer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udpServer.bind(addr)
    while True:
        data, addr = udpServer.recvfrom(1024)
        msg = str(data, encoding = 'utf-8')
        print(msg)
        if msg == "YAHBOOMRASPBOT_FIND":
            udpServer.sendto(bytes("Raspbot_Pi_V2.0", encoding='utf-8'), addr)
            print("send ok")

        #time.sleep(0.5)