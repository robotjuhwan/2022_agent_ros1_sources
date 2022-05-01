#include <ros/package.h>
#include <std_msgs/Int8.h>
#include "hero_msgs/hero_agent_state.h"
#include "hero_msgs/hero_command.h"
#include "hero_msgs/hero_agent_vision.h"
#include "ros/ros.h" // ROS Default Header File

#include <stdio.h>
#include <iostream>
#include <string>

#include <termios.h>

using namespace std;

float Yaw = 0;
float Target_yaw = 0;
int Throttle = 0;
int Valid_yaw = 0;
float Depth = 0;
float Target_depth = 0;
int Move_speed = 0;
int Cont_state = 0;
int State_addit = 0;

int cont_Yaw = 0;
int cont_Depth = 0;
int state_Relay = 0;
int state_Laser = 0;
int state_recovery = 0;

ros::Publisher pub_command;
std_msgs::Int8 command_msg;

ros::Publisher pub_vision_command;
std_msgs::Int8 vision_command_msg;

hero_msgs::hero_command srv;

// for control mission
int control_process = 0;
float start_target_depth = 0;
int process_count = 0;
int left_check_finish = 0;
int gripper_state = 0;
int sum_laser = 0;

int start_recovery = 0;

int debugging = 0;

///////////////////////////////

// for vision detected
int WHITE_VALID = 0;
int BLACK_VALID = 0;
int OBJECT_VALID = 0;
int LASER_VALID = 0;
int HIGH_LASER = 0;
int LOW_LASER = 0;
int FOR_YAW = 0;
////////////////////

void msgCallback_hero_vision(const hero_msgs::hero_agent_vision::ConstPtr &msg)
{
    WHITE_VALID = msg->WHITE_VALID;
    BLACK_VALID = msg->BLACK_VALID;
    OBJECT_VALID = msg->OBJECT_VALID;
    LASER_VALID = msg->LASER_VALID;
    HIGH_LASER = msg->HIGH_LASER;
    LOW_LASER = msg->LOW_LASER;
    FOR_YAW = msg->FOR_YAW;
}

void msgCallback_state(const hero_msgs::hero_agent_state::ConstPtr &msg)
{
    // control_process_loop

    Yaw = msg->Yaw;
    Target_yaw = msg->Target_yaw;
    Throttle = msg->Throttle;
    Valid_yaw = msg->Valid_yaw;
    Depth = msg->Depth;
    Target_depth = msg->Target_depth;
    Move_speed = msg->Move_speed;
    Cont_state = msg->Cont_state;
    State_addit = msg->State_addit;
    std::system("clear");
    ROS_INFO("Yaw msg = %f", msg->Yaw);                   // Prints the 'stamp.sec' message
    ROS_INFO("Target_yaw msg = %f", msg->Target_yaw);     // Prints the 'stamp.nsec' message
    ROS_INFO("Throttle msg = %d", msg->Throttle);         // Prints the 'data' message
    ROS_INFO("Valid_yaw msg = %d", msg->Valid_yaw);       // Prints the 'data' message
    ROS_INFO("Depth msg = %f", msg->Depth);               // Prints the 'data' message
    ROS_INFO("Target_depth msg = %f", msg->Target_depth); // Prints the 'data' message
    ROS_INFO("Move_speed msg = %d", msg->Move_speed);     // Prints the 'data' message
    ROS_INFO("Cont_state msg = %d", msg->Cont_state);     // Prints the 'data' message

    if (State_addit % 2 == 1)
    {
        cont_Yaw = 1;
    }
    else
    {
        cont_Yaw = 0;
    }
    State_addit = State_addit / 2;
    if (State_addit % 2 == 1)
    {
        cont_Depth = 1;
    }
    else
    {
        cont_Depth = 0;
    }
    State_addit = State_addit / 2;
    if (State_addit % 2 == 1)
    {
        state_Relay = 1;
    }
    else
    {
        state_Relay = 0;
    }
    State_addit = State_addit / 2;
    if (State_addit % 2 == 1)
    {
        state_Laser = 1;
    }
    else
    {
        state_Laser = 0;
    }
    State_addit = State_addit / 2;
    if (State_addit % 2 == 1)
    {
        state_recovery = 1;
    }
    else
    {
        state_recovery = 0;
    }

    ROS_INFO("cont_Yaw msg = %d", cont_Yaw);             // Prints the 'data' message
    ROS_INFO("cont_Depth msg = %d", cont_Depth);         // Prints the 'data' message
    ROS_INFO("state_Relay msg = %d", state_Relay);       // Prints the 'data' message
    ROS_INFO("state_Laser msg = %d", state_Laser);       // Prints the 'data' message
    ROS_INFO("state_recovery msg = %d", state_recovery); // Prints the 'data' message

    ROS_INFO("WHITE_VALID msg = %d", WHITE_VALID);   // Prints the 'data' message
    ROS_INFO("BLACK_VALID msg = %d", BLACK_VALID);   // Prints the 'data' message
    ROS_INFO("OBJECT_VALID msg = %d", OBJECT_VALID); // Prints the 'data' message
    ROS_INFO("LASER_VALID msg = %d", LASER_VALID);   // Prints the 'data' message
    ROS_INFO("HIGH_LASER msg = %d", HIGH_LASER);     // Prints the 'data' message
    ROS_INFO("LOW_LASER msg = %d", LOW_LASER);       // Prints the 'data' message
    ROS_INFO("FOR_YAW msg = %d", FOR_YAW);

    ROS_INFO("control_process %d", control_process); // Prints the 'data' message
    ROS_INFO("process_count %d", process_count);     // Prints the 'data' message

    ROS_INFO("debugging %d", debugging); // Prints the 'data' message

    if (control_process == 0)
    {
        control_process = 0;
        start_target_depth = 0;
        process_count = 0;
        left_check_finish = 0;
        gripper_state = 0;
        start_recovery = 0;

        if (msg->Cont_state != 0) // stop
        {
            command_msg.data = 'q';
            pub_command.publish(command_msg);
        }
    }
    else if (control_process == 1) // start position
    {
        // yaw cont, depth cont on
        // save init_depth
        if (cont_Yaw == 0)
        {
            command_msg.data = 'y';
            pub_command.publish(command_msg);
        }
        else if (cont_Depth == 0)
        {
            command_msg.data = 'p';
            pub_command.publish(command_msg);
        }
        start_target_depth = msg->Target_depth;
        process_count = 0;
        left_check_finish = 0;
    }
    else if (control_process == 2) // go forward until white and black detected
    {

        if (msg->Move_speed < 30) // s
        {
            command_msg.data = 'z';
            pub_command.publish(command_msg);
        }
        else if (msg->Move_speed > 30) // s
        {
            command_msg.data = 'x';
            pub_command.publish(command_msg);
        }

        if (process_count == 0) // stop
        {
            start_target_depth = start_target_depth;
        }
        process_count++;

        if (msg->Target_depth > start_target_depth && process_count % 20 == 0) // stop
        {
            command_msg.data = ',';
            pub_command.publish(command_msg);
        }

        if (BLACK_VALID >= 1 && WHITE_VALID == 1)
        {
            if (msg->Cont_state != 0) // stop
            {
                command_msg.data = 'q';
                pub_command.publish(command_msg);
            }
            else
            {
                process_count = 0;
                control_process = 3;
            }
        }
        else if (msg->Cont_state != 1) // forward
        {
            command_msg.data = 's';
            pub_command.publish(command_msg);
        }
    }
    else if (control_process == 3) // tracking the init black area
    {
        vision_command_msg.data = '7';
        pub_vision_command.publish(vision_command_msg);

        if (msg->Cont_state != 5) // stop
        {
            command_msg.data = '1';
            pub_command.publish(command_msg);
        }

        process_count++;
        if (process_count > 200)
        {
            process_count = 0;
            control_process = 4;
        }
    }
    else if (control_process == 4) // check left black
    {
        vision_command_msg.data = '-';
        pub_vision_command.publish(vision_command_msg);

        if (msg->Cont_state != 5) // stop
        {
            command_msg.data = '1';
            pub_command.publish(command_msg);
        }

        process_count++;

        if (BLACK_VALID == 2)
        {
            process_count = 0;
            control_process = 5;
        }

        if (process_count > 100)
        {
            process_count = 0;
            control_process = 5;
            left_check_finish = 1;
        }
    }
    else if (control_process == 5) // go left
    {
        vision_command_msg.data = '9';
        pub_vision_command.publish(vision_command_msg);

        if (msg->Cont_state != 5) // stop
        {
            command_msg.data = '1';
            pub_command.publish(command_msg);
        }

        process_count++;
        if (process_count > 200)
        {
            if (left_check_finish == 1)
            {
                process_count = 0;
                control_process = 6;
                sum_laser = 0;
            }
            else
            {
                process_count = 0;
                control_process = 4;
            }
        }
    }
    else if (control_process == 6) // check the black
    {
        process_count++;
        if (state_Laser != 1 && process_count < 5) //
        {
            command_msg.data = 'r';
            pub_command.publish(command_msg);
            process_count = 0;
        }
        if (process_count > 30)
        {

            sum_laser = sum_laser / 20;
            debugging = sum_laser;
            if (sum_laser < 15)
            {
                process_count = 0;
                control_process = 7;
            }
            else if (sum_laser > 20)
            {
                process_count = 0;
                control_process = 9;
            }
            else
            {
                process_count = 0;
                sum_laser = 0;
            }
        }

        else if (process_count > 25)
        {
            if (state_Laser != 0) //
            {

                command_msg.data = 'f';
                pub_command.publish(command_msg);
            }
        }
        else if (process_count > 5)
        {
            sum_laser = sum_laser + HIGH_LASER - LOW_LASER;
            ROS_INFO("LASER Different = %d", +HIGH_LASER - LOW_LASER); // Prints the 'data' message
        }
    }
    else if (control_process == 7) // check right
    {
        if (state_Laser != 0) //
        {
            sum_laser = 0;
            command_msg.data = 'f';
            pub_command.publish(command_msg);
        }

        vision_command_msg.data = '=';
        pub_vision_command.publish(vision_command_msg);

        if (msg->Cont_state != 5) // stop
        {
            command_msg.data = '1';
            pub_command.publish(command_msg);
        }

        process_count++;

        if (BLACK_VALID == 2 && process_count > 20)
        {
            process_count = 0;
            control_process = 8;
        }

        if (process_count > 100)
        {
            process_count = 0;
            control_process = 8;
        }
    }
    else if (control_process == 8) //
    {

        vision_command_msg.data = '7';
        pub_vision_command.publish(vision_command_msg);

        if (msg->Cont_state != 5) // stop
        {
            command_msg.data = '1';
            pub_command.publish(command_msg);
        }
        process_count++;
        if (process_count > 200)
        {
            process_count = 0;
            control_process = 6; // mount object
            gripper_state = 0;
        }
    }
    else if (control_process == 9) // mount object
    {
        if (state_Laser != 0) //
        {
            sum_laser = 0;
            command_msg.data = 'f';
            pub_command.publish(command_msg);
        }

        if (gripper_state == 0)
        {
            command_msg.data = 'c';
            pub_command.publish(command_msg);
        }
        else if (gripper_state == 1)
        {
            command_msg.data = 'v';
            pub_command.publish(command_msg);
        }

        process_count++;
        if (process_count > 50)
        {
            gripper_state++;
        }

        if (gripper_state > 1)
        {
            process_count = 0;
            control_process = 10;
        }
    }
    else if (control_process == 10) // check right
    {

        vision_command_msg.data = '=';
        pub_vision_command.publish(vision_command_msg);

        if (msg->Cont_state != 5) // stop
        {
            command_msg.data = '1';
            pub_command.publish(command_msg);
        }

        process_count++;

        if (BLACK_VALID == 2)
        {
            process_count = 0;
            control_process = 11;
        }

        if (process_count > 100)
        {
            process_count = 0;
            control_process = 11;
        }
    }
    else if (control_process == 11) //
    {

        vision_command_msg.data = '7';
        pub_vision_command.publish(vision_command_msg);

        if (msg->Cont_state != 5) // stop
        {
            command_msg.data = '1';
            pub_command.publish(command_msg);
        }
        process_count++;
        if (process_count > 200)
        {
            process_count = 0;
            control_process = 12; // mount object
        }
    }
    else if (control_process == 12) // access
    {
        vision_command_msg.data = '8';
        pub_vision_command.publish(vision_command_msg);

        process_count++;
        if (process_count > 20) // stop
        {
            process_count = 0;
            command_msg.data = '.';
            pub_command.publish(command_msg);

            if (msg->Target_depth >= Depth + 0.05)
            {
                control_process = 13;
                gripper_state = 0;
                process_count = 0;
            }
        }
    }
    else if (control_process == 13) // grip
    {
        if (gripper_state == 0)
        {
            command_msg.data = 'b';
            pub_command.publish(command_msg);
        }
        else if (gripper_state == 1)
        {
            command_msg.data = 'v';
            pub_command.publish(command_msg);
        }

        process_count++;
        if (process_count > 50)
        {
            gripper_state++;
        }

        if (gripper_state > 1)
        {
            process_count = 0;
            control_process = 14;
        }
    }
    else if (control_process == 14) // go init_depth
    {
        process_count++;
        if (msg->Target_depth > start_target_depth && process_count > 10) // stop
        {
            process_count = 0;
            command_msg.data = ',';
            pub_command.publish(command_msg);
        }

        if (msg->Target_depth <= start_target_depth)
        {
            process_count = 0;
            control_process = 15;
            gripper_state = 0;
        }
    }
    else if (control_process == 15) // go init_depth
    {
        process_count++;
        if (process_count < 30)
        {
            if (msg->Cont_state != 3) // left
            {
                command_msg.data = 'a';
                pub_command.publish(command_msg);
            }
        }
        else
        {
            if (msg->Cont_state != 2) // forward
            {
                command_msg.data = 'w';
                pub_command.publish(command_msg);
            }
        }

        if (BLACK_VALID == 0)
        {
            process_count = 0;
            control_process = 16;
            start_recovery = 0;
        }
    }
    else if (control_process == 16) // go init_depth
    {

        if (state_recovery != 1) // left
        {
            command_msg.data = '2';
            pub_command.publish(command_msg);
        }

        if (state_recovery == 1) // left
        {
            start_recovery = 1;
        }

        if (start_recovery == 1 && state_recovery == 0) // left
        {

            process_count = 0;
            control_process = 0;
        }
    }
}

// for keyboard input
static struct termios initial_settings, new_settings;

static int peek_character = -1;

void init_keyboard();
void close_keyboard();
int _kbhit();
int _getch();
int _putch(int c);
///////////////////////////////////

int main(int argc, char **argv) // Node Main Function
{
    cout << "Start!" << endl;
    int count = 0;

    ros::init(argc, argv, "agent_main"); // Initializes Node Name
    ros::NodeHandle
        nh;                   // Node handle declaration for communication with ROS system
    ros::Rate loop_rate(100); // Hz

    ros::Subscriber sub_state =
        nh.subscribe("/hero_agent/state", 100, msgCallback_state);

    ros::Subscriber sub_vision =
        nh.subscribe("/hero_agent/vision", 100, msgCallback_hero_vision);

    pub_command =
        nh.advertise<std_msgs::Int8>("/hero_agent/command", 100);

    pub_vision_command =
        nh.advertise<std_msgs::Int8>("/hero_agent/vision_command", 100);

    float degree = 0;

    char pc_input, qq;

    init_keyboard();

    // 시리얼 포트를 연다
    // 시리얼포트를 1초동안 대기하거나 32바이트 이상의 데이타가 들어오면
    // 깨어나도록 설정한다.

    cout << "real Start!" << endl;
    while (ros::ok())
    {
        if (_kbhit())
        {

            int ch = _getch();

            _putch(ch);
/*
            if (ch == '6' || ch == '7' || ch == '8' || ch == '9' || ch == '-' || ch == '=')
            {
                cout << "srv" << endl;
                vision_command_msg.data = ch;
                pub_vision_command.publish(vision_command_msg);
            }
            else if (ch == '4' || ch == '5')
            {
                if (ch == '4')
                {
                    if (control_process > 0)
                    {
                        control_process--;
                    }
                }
                else if (ch == '5')
                {
                    if (control_process < 20)
                    {
                        control_process++;
                    }
                }
            }
            else
            {*/
                command_msg.data = ch;
                pub_command.publish(command_msg);
            //}
            // agent_command_srv_cli.call(srv);
            // ROS_INFO("send srv, vision_command_msg.data: %d", vision_command_msg.data);
            // ROS_INFO("receive srv, srv.Response.result: %d", srv.response.result);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
    ros::spin(); // Increase count variable by one

    close_keyboard();
    return 0;
}

void init_keyboard()
{
    tcgetattr(0, &initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
}

void close_keyboard()
{
    tcsetattr(0, TCSANOW, &initial_settings);
}

int _kbhit()
{
    unsigned char ch;
    int nread;

    if (peek_character != -1)
        return 1;

    new_settings.c_cc[VMIN] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0, &ch, 1);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);

    if (nread == 1)
    {
        peek_character = ch;
        return 1;
    }

    return 0;
}

int _getch()
{
    char ch;

    if (peek_character != -1)
    {
        ch = peek_character;
        peek_character = -1;

        return ch;
    }
    read(0, &ch, 1);

    return ch;
}

int _putch(int c)
{
    putchar(c);
    fflush(stdout);

    return c;
}