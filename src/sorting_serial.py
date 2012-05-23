#!/usr/bin/python
import sys
import time
import glob

## First thing that we need to do is import the serial library:
try:
    import serial
except ImportError:
    print "[FATAL] Serial library not found"
    sys.exit(0)

## delay time:
delaytime = 0.05

def read_com(port='/dev/ttyUSB0'):
    try:
        com = serial.Serial(port,
                            baudrate=115200,
                            timeout=1,
                            stopbits=2,
                            parity=serial.PARITY_NONE,
                            rtscts=False)
    except:
        print "Could not open serial port",port+"!"
        sys.exit(1)
        
        
    out = "+++"
    com.write(out)
    time.sleep(delaytime)
    data1 = com.read(3)
    if not (data1):
        print "Failed to enter command mode on ",port
        sys.exit(1)
    ## print "sent = ",out
    ## print "rec = ",data1

    out = "ATNI\r"
    com.write(out)
    time.sleep(delaytime)
    data2 = com.read(2)
    if not (data2):
        print "Failed to get id ",port
        sys.exit(1)
    ## print "sent = ",out
    ## print "rec = ",data2

    out = "ATCN\r"
    com.write(out)
    time.sleep(delaytime)
    data3 = com.read(3)
    if not (data3):
        print "Failed to close command mode ",port
        sys.exit(1)
    ## print "sent = ",out
    ## print "rec = ",data3

    ## close serial port:
    com.close()

    return int(data2)



if __name__ == "__main__":
    ports = glob.glob('/dev/ttyUS*')

    dat = []
    for p in ports:
        num = read_com(p)
        dat.append((p, num))

    for v in dat:
        print v
        

    
    
