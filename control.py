import serial, json, time, threading

ser = serial.Serial(
    port='/dev/ttyACM0', 
    baudrate=115200, 
    bytesize=8, 
    parity="N", 
    stopbits=1.0, 
    timeout=1)

positions = {"A1":100, "A2":0, "A3":0, "A4":0, "A5":0, "A6":0}

state = True

def send():
    while state:
        ser.write((json.dumps(positions) + "\n").encode())
        print(f"\rSending: {positions}", end="")
        time.sleep(0.1)

threading.Thread(target=send, daemon=True).start()

try:
    while True:
        cmd = input("\Enter positions: ").strip()
        if cmd.lower() == "stop":
            state = False
            break
        try:
            joint, val =cmd.split()
            val= float(val)
            if joint in positions:
                positions[joint]=val
            else:
                print("Unknown joint name. ")
        except ValueError:
            print("Usage: A2-A6 or On 1")
except KeyboardInterrupt:
    state = False
