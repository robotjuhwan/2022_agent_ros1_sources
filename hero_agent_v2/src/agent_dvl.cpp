#include <ros/package.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#include "hero_msgs/hero_agent_dvl.h"
#include "ros/ros.h" // ROS Default Header File

#include <stdio.h>
#include <iostream>
#include <string>

#include <termios.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define BAUDRATE B115200
#define MODEMDEVICE "/dev/ttyUSB1"
#define _POSIX_SOURCE 1 /* POSIX 호환 소스 */

using namespace std;

int fd;

void msgCallback_command(const std_msgs::Int8::ConstPtr &msg)
{
    int8_t command = msg->data;

    if (command == 0)
    {
        unsigned char com[] = {'w', 'c', 'v', '\r', '\n'};
        write(fd, com, sizeof(com));
        printf("%s", com);
    }
    else if (command == 1)
    {
        unsigned char com[] = {'w', 'c', 'r', '\r', '\n'};
        write(fd, com, sizeof(com));
        printf("%s", com);
    }
    else if (command == 2)
    {
        unsigned char com[] = {'w', 'c', 's', ',', ',', ',', 'y', ',', '\r', '\n'};
        write(fd, com, sizeof(com));
        printf("%s", com);
    }
    else if (command == 3)
    {
        unsigned char com[] = {'w', 'c', 's', ',', ',', ',', 'n', ',', '\r', '\n'};
        write(fd, com, sizeof(com));
        printf("%s", com);
    }
    else if (command == 4)
    {
        unsigned char com[] = {'w', 'c', 's', ',', ',', ',', ',', 'y', '\r', '\n'};
        write(fd, com, sizeof(com));
        printf("%s", com);
    }
    else if (command == 5)
    {
        unsigned char com[] = {'w', 'c', 's', ',', ',', ',', ',', 'n', '\r', '\n'};
        write(fd, com, sizeof(com));
        printf("%s", com);
    }
}

int main(int argc, char **argv) // Node Main Function
{
    int c, res;
    struct termios oldtio, newtio;
    char buf[255];

    double time_stamp, dvl_x, dvl_y, dvl_z, pos_std, roll, pitch, yaw;
    int status;

    fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror(MODEMDEVICE);
        exit(-1);
    }

    tcgetattr(fd, &oldtio);         /* save current serial port settings */
    bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR | ICRNL;
    newtio.c_oflag = 0;
    newtio.c_lflag = ICANON;

    newtio.c_cc[VINTR] = 0;    /* Ctrl-c */
    newtio.c_cc[VQUIT] = 0;    /* Ctrl-\ */
    newtio.c_cc[VERASE] = 0;   /* del */
    newtio.c_cc[VKILL] = 0;    /* @ */
    newtio.c_cc[VEOF] = 4;     /* Ctrl-d */
    newtio.c_cc[VTIME] = 0;    /* inter-character timer unused */
    newtio.c_cc[VMIN] = 1;     /* blocking read until 1 character arrives */
    newtio.c_cc[VSWTC] = 0;    /* '\0' */
    newtio.c_cc[VSTART] = 0;   /* Ctrl-q */
    newtio.c_cc[VSTOP] = 0;    /* Ctrl-s */
    newtio.c_cc[VSUSP] = 0;    /* Ctrl-z */
    newtio.c_cc[VEOL] = 0;     /* '\0' */
    newtio.c_cc[VREPRINT] = 0; /* Ctrl-r */
    newtio.c_cc[VDISCARD] = 0; /* Ctrl-u */
    newtio.c_cc[VWERASE] = 0;  /* Ctrl-w */
    newtio.c_cc[VLNEXT] = 0;   /* Ctrl-v */
    newtio.c_cc[VEOL2] = 0;    /* '\0' */

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    ros::init(argc, argv, "agent_dvl"); // Initializes Node Name
    ros::NodeHandle nh;                 // Node handle declaration for communication with ROS system
    ros::Rate loop_rate(100);           // Hz

    ros::Subscriber sub_command =
        nh.subscribe("/hero_agent/dvl_command", 100, msgCallback_command);

    ros::Publisher pub_dvl;
    hero_msgs::hero_agent_dvl dvl_msg;

    pub_dvl =
        nh.advertise<hero_msgs::hero_agent_dvl>("/hero_agent/dvl", 100);

    ros::Publisher pub_dvl_raw;
    std_msgs::String dvl_raw_msg;

    pub_dvl_raw =
        nh.advertise<std_msgs::String>("/hero_agent/dvl_raw", 100);

    cout << "Start!" << endl;
    while (ros::ok())
    {
        res = read(fd, buf, 255);
        buf[res] = 0; /* set end of string, so we can printf */
        if (buf[0] == 'w' && buf[1] == 'r' && buf[2] == 'p')
        {
            printf(":%s:%d\n", buf, res);
            sscanf(buf, "wrp,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d*de", &time_stamp, &dvl_x, &dvl_y, &dvl_z, &pos_std, &roll, &pitch, &yaw, &status);

            dvl_msg.TIME_STAMP = time_stamp;
            dvl_msg.X = dvl_x;
            dvl_msg.Y = dvl_y;
            dvl_msg.Z = dvl_z;
            dvl_msg.POS_STD = pos_std;
            dvl_msg.ROLL = roll;
            dvl_msg.PITCH = pitch;
            dvl_msg.YAW = yaw;
            dvl_msg.VALID = status;

            pub_dvl.publish(dvl_msg);
        }
        else if (buf[0] == 'w' && buf[1] == 'r' && buf[2] == 'z')
        {
            printf(":%s:%d\n", buf, res);

            string str(buf);

            dvl_raw_msg.data = str;
            pub_dvl_raw.publish(dvl_raw_msg);
        }
        else if (buf[0] == 'w' && buf[1] == 'r' && (buf[2] == 'a' || buf[2] == 'n' || buf[2] == 'v'))
        {
            printf(":%s:%d\n", buf, res);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    ros::spin(); // Increase count variable by one

    tcsetattr(fd, TCSANOW, &oldtio);

    return 0;
}