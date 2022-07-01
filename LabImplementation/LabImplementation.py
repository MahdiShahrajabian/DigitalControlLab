import serial
import getch
serialport = serial.Serial ("/dev/ttyS0")
serialport.baudrate = 115200
while(True):
    x = getch.getch().lower()
    if x=='q':
        break
    elif x=='w':
        cmd = "+100+10015+00"
    elif x=='s':
        cmd = "-100-10015+00"
    elif x == 'd':
        cmd = "+050-05015+00"
    elif x == 'a':
        cmd = "-050+05015+00"
    else:
        cmd = "+000+00015+00"
    print(x)
    serialport.write(cmd.encode())
