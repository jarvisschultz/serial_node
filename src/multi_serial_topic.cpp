/*******************************************************************************
  Jarvis Schultz and Jake Ware

  This ros node is a service node for clients that publish topics of
  type robot_command.  Each robot command topic has a message in it
  defined in command_msgs.  This code interprets the type of message,
  converts the data in the message into a format that the robot can
  understand, and then it sends the data out on the serial port, and
  then responds to the client with an acknowledgement.

*******************************************************************************/
  
//Notes:

  
/*****INCLUDES ****************************************************************/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <float.h>
#include <time.h>
#include <assert.h>
#include <kbhit.h>
#include <iostream>

#include "puppeteer_msgs/RobotCommands.h"

/*****DEFINITIONS *************************************************************/
#define	BAUDRATE		B115200
#define	MODEMDEVICE		"/dev/ttyUSB0"
#define _POSIX_SOURCE		1 /* POSIX compliant source */
#define SHORT_PACKET_SIZE	3  // bytes
#define PACKET_SIZE		12
#define	LONG_PACKET_SIZE	(18)
#define MAX_ROBOTS		(9)
#define MAX_FLOATS		(5)
#define BYTES_PER_FLOAT		(3)


/*****DECLARATIONS ************************************************************/
void sendData(int, unsigned char *, unsigned int);
void initComm(std::string);
void BuildNumber(unsigned char *, float, short int);
void MakeString(unsigned char *, char, float*, int, int);
void stopRobots(void);
bool GetData(const unsigned char, const int, float *, float *, float *);
int DataChecker(const unsigned char, const unsigned char *);
int ReadSerial(unsigned char *);
float InterpNumber(const unsigned char *);
void keyboardcb(const ros::TimerEvent &);
int get_key(int);


/*****GLOBALS *****************************************************************/
int fd;
bool exit_flag;
bool transfer_flag;
struct termios oldtio,newtio;
struct timespec stopdelay = {0, 200000000};
struct timespec delayrem;
unsigned char packet_prev[128];
// Define publisher for publishing the value handled by
// serial_node.  We do this because we cannot bag service calls
// currently
ros::Publisher pub[MAX_ROBOTS];
ros::Subscriber sub[MAX_ROBOTS];
ros::Subscriber keyboard_sub;
int nr = 0;


/*****CALLBACKS ***************************************************************/
// This function gets called if we have detected that the user is pressing a key
// on the keyboard.
void keyboardcb(const ros::TimerEvent& e) 
{
    ROS_DEBUG("keyboardcb triggered\n");
      // check kbhit() to see if there was a keyboard strike and
    // transfer_flag to see if there is a node sending serial data
    if(kbhit()) 
    {
	ROS_DEBUG("Keyboard Strike Detected\n");
	int c = fgetc(stdin);

	if(c == 'p')
	{
	    ROS_WARN("Reopening Serial Access");
	    exit_flag = false;
	}
	else
	{
	    ROS_WARN("Stopping Robots and Closing Serial Access"); 
	    ros::param::set("/operating_condition", 4);
	    exit_flag = true;
	}
    }
    if(exit_flag == true) stopRobots();
}


// this function is responsible for parsing the incoming serial
// requests, building the correct string, and sending it out to the
// robots.
void send_command_cb(const puppeteer_msgs::RobotCommands& c)
{
    ROS_DEBUG("Serial request received"); 
    char type = c.type;
    unsigned char out[128];
    float vals[MAX_FLOATS];
    int len = 0;
    memset(vals,0,sizeof(vals));
    
    switch (type)
    {
    case 'h': // MOT_SPEED
	len = PACKET_SIZE;
	vals[0] = c.v_left;
	vals[1] = c.v_right;
	vals[2] = c.v_top;	
	break;
    case 'm': // START
	len = PACKET_SIZE;
	break;
    case 'd': // EXT_SPEED
	len = PACKET_SIZE;
	vals[0] = c.v_robot;
	vals[1] = c.w_robot;
	vals[2] = c.rdot;
	break;
    case 'l':
	len = PACKET_SIZE;
	vals[0] = c.x;
	vals[1] = c.y;
	vals[2] = c.th;
    }


    MakeString(out, type, vals, len, c.div);
    sendData(c.robot_index, out, len);
    
    return;
}


/*****FUNCTIONS ***************************************************************/

void MakeString(unsigned char *dest, char type, float *vals,
		int num, int div)
{
    *dest = type;
    for (int j=0; j<num; j++)
	BuildNumber((dest+1+j*BYTES_PER_FLOAT), vals[j], div);
    return;
}



void sendData(int id, unsigned char *DataString, unsigned int len)
{
    char packet[128];
    unsigned int i = 0;
    unsigned short address =  0;
    unsigned int checksum = 0;

    // Initialize the packet to all zeros:
    memset(packet,0,sizeof(packet));

    address = id;

    // Now we can begin filling in the packet:
    packet[0] = DataString[0];
    sprintf(&packet[1],"%1d",address);
    for(i = 2; i < len-1; i++)
	packet[i] = DataString[i-1];

    // Now, let's calculate a checksum:
    checksum = 0;
    for(i = 0; i < len-1; i++)
	checksum += packet[i];

    checksum = 0xFF-(checksum & 0xFF);

    packet[len-1] = checksum;    
      
    write(fd, packet, len);
    fsync(fd);
    ROS_INFO("Sending String to Robot %d:", id);
    for(i = 0; i < len; i++)
	printf("%02X ",(unsigned char) packet[i]);
    printf("\n");
}


// this function takes in a robot index (e.g. 49 == 0x31 == '1') and
// returns the robot name that has that index
int get_key(int index)
{
    ROS_DEBUG("Getting KEY");
    int tmp = 0;
    for (int i=0; i<nr; i++)
    {
	std::stringstream ss;
	ss << "/robot_" << i+1 << "/robot_index";
	if(ros::param::has(ss.str()))
	{
	    ros::param::get(ss.str(), tmp);
	}
	else
	    ROS_WARN("Could not find parameter");

	if (tmp == index)
	    return i+1;	    
    }

    return 0;
}



    
void stopRobots(void)
{
    unsigned char szBufferToTransfer[16];
    static int robot_id = 9;
    float vals[3] = {0.0,0.0,0.0};
  
    // Let's make the data string:
    MakeString(szBufferToTransfer, 'q', vals, 3, 3);
    // Now let's send out the data string:
    sendData(robot_id, szBufferToTransfer, PACKET_SIZE);

    // Set the robot_id value for the next call of this function:
    if (robot_id == 9) robot_id = 1;
    else if (robot_id == 3) robot_id = 9;
    else robot_id++;
}



// The following function is used for opening a com port to
// communicate with the mobile robot.
void initComm(std::string dev)
{
    /* 
       Open modem device for reading and writing and not as controlling tty
       because we don't want to get killed if linenoise sends CTRL-C.
    */
    fd = open(dev.c_str(), O_RDWR | O_NOCTTY ); 
    if (fd <0) {perror(MODEMDEVICE); exit(-1); }

    tcgetattr(fd,&oldtio); /* save current serial port settings */
    bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */
 
    /* 
       BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
       CRTSCTS : output hardware flow control (only used if the cable has
       all necessary lines. See sect. 7 of Serial-HOWTO)
       CS8     : 8n1 (8bit,no parity)
       CSTOPB  : enable 2 stop bits
       CLOCAL  : local connection, no modem contol
       CREAD   : enable receiving characters
    */
    newtio.c_cflag = BAUDRATE /* | CRTSCTS */ | CS8 | CLOCAL | CREAD | CSTOPB;
	 
    /*
      IGNPAR  : ignore bytes with parity errors
      ICRNL   : map CR to NL (otherwise a CR input on the other computer
      will not terminate input)
      otherwise make device raw (no other input processing)
    */
    newtio.c_iflag = IGNPAR;

    /*
      Raw output.
    */
    newtio.c_oflag &= ~OPOST;

    /*
     * Disable canonical input so that we can read byte-by-byte 
     */
    newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    /* Now we need to set the number of bytes that we want to read in,
     * and the timeout length */
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
	
    /* 
       now clean the modem line and activate the settings for the port
    */
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);
}


void BuildNumber(unsigned char *destination, float value, short int divisor)
{
    int valint = 0;
    int i = 0;
    short int i1;
    short unsigned int i2, i3;
    // First thing is to move the decimal point of the integer to the
    // right a "divisor" number of times:
    for(i = 0; i<divisor; i++) value = value*10.0;
    valint = (int) value;
    // Now build the three chars:
    i1 = ((valint<<3) & 0xFF0000)>>16;
    i2 = ((valint<<3) & 0x00FF00)>>8;
    i3 = ((valint<<3) & 0x0000FF);
    i3 = (((i3)&0xF0)) + ((i3&0x0F)|divisor);

    // Now, place the chars in the array:
    *(destination) = i1;
    *(destination+1) = i2;
    *(destination+2) = i3;
        
    return;
}


bool GetData(const unsigned char type, const int id, float *val1, float *val2, float *val3)
{
    bool read_flag = true;
    unsigned char szPtr[128];
    unsigned char data[128];
    int i = 0;
    memset(szPtr, 0, sizeof(szPtr));
    memset(data, 0, sizeof(data));
        
    // First we need to send out the request for the robot to send its
    // data:
    szPtr[0] = type;
    szPtr[1] = (id+0x30);
    szPtr[2] = 0xFF-((type+id+0x30) & 0xFF);
    
    write(fd, szPtr, SHORT_PACKET_SIZE);
    fsync(fd);
    ROS_INFO("Sending request string to Robot %d:", id);
    for(i = 0; i < SHORT_PACKET_SIZE; i++)
	printf("%02X ",szPtr[i]);
    printf("\n");
    
    // Now, we need to get the data sent back:
    if(ReadSerial(data))
    {
	// Good read! Let's check validity:
	if(DataChecker(type, data))
	{
	    read_flag = false;
	    // If it is good data, let's interpret it:
	    *val1 = InterpNumber(&data[2]);
	    *val2 = InterpNumber(&data[5]);
	    *val3 = InterpNumber(&data[8]);
	    return read_flag;
	}
	else
	    ROS_WARN("Full string received, but not valid");
    }
    else
	ROS_WARN("Not enough data received");

    // Then let's estimate the data:
    *val1 = 0.0;
    *val2 = 0.0;
    *val3 = 0.0;
    
    return read_flag;
}    

int DataChecker(const unsigned char type, const unsigned char *data)
{
    short unsigned int checksum = 0;
    int i = 0;
    int validity = 0;
    // First, is the first character a valid reply character:
    if (data[0] == type)
    {
	// Now check the address:
	if (data[1] == '0')
	{
	    // Now check the checksum:
	    for (i=0; i<PACKET_SIZE-1; i++)
		checksum += data[i];
	    checksum = 0xFF-(checksum & 0xFF);
	    if (checksum == (unsigned int) data[PACKET_SIZE-1])
	    {
		validity = 1;
	    }
	}
    }
    return validity;
}

int ReadSerial(unsigned char *data)
{
    unsigned char *bufptr;      /* Current char in buffer */
    int  nbytes = 0;	      /* Number of bytes read */
    short unsigned int count = 0;
    int bytes_read = 0;
    int successful_read = 0;
    struct timespec newdelay = {0, 100000};
    struct timespec delayrem;

    bufptr = data;

    // Let's try to read in the sent command a few times:
    for (count = 0; count < 10*PACKET_SIZE; count++)
    {
	if((nbytes = read(fd, bufptr, PACKET_SIZE)) == -1)
	    ROS_WARN("Read Error!");
      
	bytes_read += nbytes;
	bufptr += nbytes;
	if (bytes_read >= PACKET_SIZE)
	{
	    ROS_INFO("Received: ");
	    for (nbytes = 0; nbytes < 12; nbytes++)
		printf("%02X ", (unsigned char) data[nbytes]);
	    printf("\n");
	    successful_read = 1;
	    break;
	}
	if(nanosleep(&newdelay,&delayrem)) printf("Nanosleep Error\n");;
    }
    tcflush(fd, TCIOFLUSH);
    return successful_read;
}
		

float InterpNumber(const unsigned char *data)
{
    unsigned int num1 = 0;
    int num2 = 0;
    float numf = 0.0;
    short int c1 = 0;
    short unsigned int c2, c3;
    short unsigned int divisor = 1;

    // First, let's get the numeric values of each char:
    c1 = (short int) *(data);
    c2 = (short unsigned int) *(data+1);
    c3 = (short unsigned int) *(data+2);
    divisor = (((short unsigned int) *(data+2)) & 0x07);
    num1 = ((((c1<<16)&0xFF0000)+((c2<<8)&0x00FF00)+(c3)))<<11;
    num2 = ((int) (num1))>>14;
    numf = ((float) num2)/(powf((float) 10.0,(float) divisor));

    return numf;
}


/*****MAIN *************************************************************/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle n;

    // Initialize communication
    std::string dev;
    if (ros::param::has("serial_device"))
    {
	ros::param::get("serial_device", dev);
    }
    else {
	dev = MODEMDEVICE;
	ROS_INFO("Using default serial device %s",dev.c_str());
	ros::param::set("serial_device", dev);
    }
    initComm(dev);

    // get the number of robots
    if (ros::param::has("/number_robots"))
	ros::param::get("/number_robots",nr);
    else
    {
	ROS_WARN("Number of robots not set...");
	ros::param::set("/number_robots", 1);
	nr = 1;
    }

    ros::Timer kb_timer = n.createTimer(ros::Duration(0.02), keyboardcb);

    // Setup publishers and subscribers:
    for (int j=0; j<nr; j++)
    {
	std::stringstream ss;
	ss << "/robot_" << j+1 << "/serviced_values";
	pub[j] = n.advertise<geometry_msgs::PointStamped> (ss.str(), 100);
	ss.str("");
	ss << "/robot_" << j+1 << "/serial_commands";
	sub[j] = n.subscribe(ss.str(), 10, send_command_cb);
    }
    keyboard_sub = n.subscribe("/keyboard_serial_commands", 10, send_command_cb);

    ROS_INFO("Starting Serial Node...");
    
    // Wait for new data:
    ros::spin();
}

