import serial
import json
import time

ser = serial.Serial("/dev/ttyACM0", 115200)

cmd = {
    "A1":0,
    "A2":-100,
    "A3":0,
    "A4":0,
    "A5":0,
    "A6":0,
    "On":1
}

ser.write((json.dumps(cmd) + "\n").encode())

time.sleep(1)

cmd["On"] = 0
ser.write((json.dumps(cmd) + "\n").encode())