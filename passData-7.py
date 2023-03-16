# Passing data from Python to Arduino
import serial
import time
import keyboard

arduinoData = serial.Serial('com3', 115200)
    # Cmd: 0->Right 1->Back 2->Forward
    #      3->Left  4->Up   5->Down
    #      6->Forward + Right 7->Forward + Left 8->Back + Right
    #      9->Back + Left     10->Stop
while True:
# myCmd = input('cmd = ')
    for i in range (30):
        time.sleep(0.9)
        # myCmd = input("cmd=")
        myCmd = str(3)
        myCmd = myCmd + '\r'
        arduinoData.write(myCmd.encode())


    # while keyboard.is_pressed('s'):
    #     myCmd = str(10)
    #     myCmd = myCmd + '\r'
    #     arduinoData.write(myCmd.encode())
    #     exit()



