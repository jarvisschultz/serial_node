#!/usr/bin/python
import sys
import os
import time
import glob
import rospy
import roslib
roslib.load_manifest('serial_node')
import re
import argparse

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


def get_robot_indices(fname):

    """
    This function takes in a filename.  It parses that filename
    (presumably) a particular launch file.  It then returns a
    dictionary of robot namespaces and their corresponding robot index
    parameters.
    """
    D = {}
    f = open(fname,'rU')
    lines = ' '
    while lines:
        lines = f.readline()
        mat = re.match("(.*)(group)(.*)(ns)(.*)(\")(.*)(\")",lines)
        if mat:
            ns = mat.groups()[-2]
            mat = None
            while not mat:
                l = f.readline()
                if not l:
                    rospy.logerr("Failed to get all indices!")
                    sys.exit(1)
                mat = re.match(
                    "(.*)(robot_index)(.*)(value)(.*)(\")(.*)(\")", l)
            val = mat.groups()[-2]
            if val.rfind('arg') == -1:
                num = int(val)
            else:
                rospy.logwarn("Could not determine index for namespace = %s"%ns)
                num = -1
            D[ns] = num
    return D
    
    


if __name__ == "__main__":
    rospy.logdebug("Starting serial sorting node")
    rospy.init_node('sorting_serial', log_level=rospy.INFO)

    # did we provide a filename?
    parser = argparse.ArgumentParser()
    parser.add_argument("file",
                        help="name of a launch file to read in")
    args = parser.parse_args()
    rospy.loginfo("Input file = %s"%os.path.abspath(args.file))
    if not os.path.exists(args.file):
        rospy.logwarn("Input file not found")
        sys.exit(1)

    # get list of current serial ports    
    ports = glob.glob('/dev/ttyUS*')
    if not ports:
        rospy.logerr("No USB serial devices found!")
        sys.exit(1)

    # get all of the node identifiers off of the XBee's
    dat = []
    rospy.loginfo("Obtaining Node Identifiers...")
    for p in ports:
        num = read_com(p)
        dat.append((p, num))

    D = get_robot_indices(args.file)
    Dout = {}
    for d in D.keys():
        for v in dat:
            if v[1] == D[d]:
                Dout[d] = list(v)
    
    # print out data
    rospy.loginfo("Found the following data:")
    for (key,val) in Dout.iteritems():
        rospy.loginfo("namespace = {0:s},   device = {1:s},   index = {2:d}".
                      format(key, val[0], val[1]))
    
    # now write a file that stores data in a text file that
    # can be read in by a launch file

    # get base directory of this package:
    cmd = 'rospack find serial_node'
    p = os.popen(cmd, "r")
    direct = p.readline()
    p.close()
    tmp = direct[0:direct.find("src")]
    tmp = os.path.join(tmp, "data", os.path.splitext(os.path.basename(args.file))[0]+
                       "_serial_device_dict.xml")
    rospy.loginfo("Saving data to %s"%tmp)
    f = open(tmp, 'w')
    for key in D.keys():
        for v in dat:
            if D[key] == v[1]:
                param = "/"+key+"/serial_device"
                f.write(param+" : "+v[0]+"\r\n")
    f.close()


    
    sys.exit(0)
