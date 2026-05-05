#import functions from the other files
import startup
import time
import vision
import imageget

shutdown = 4
cont = [True]

startup.StartupAction()

#timer checking function
def TimerCheck(s):
    soup = time.time() - s
    timeRem = shutdown - soup
    if soup < shutdown: 
        print(str(round(timeRem, 2)), " seconds remain\n")
    else:
        print("\nTimeout\n")
        cont[0] = False
        
#timer start
start = time.time()
print("Beginning ", shutdown, " second countdown\n")

while cont[0] == True:
    TimerCheck(start)
    time.sleep(1)
    hsv = imageget.gethsv()

    #call color counter
    color_counts = vision.ColorCounter(hsv)

    
    if cont[0] == False:
        break

print("\nEnding Program\n")
