#include <ros/ros.h>
#include <cstdio>
#include <termios.h>

#include "../include/biosentry/drone_controller.h"

// from: https://gist.github.com/whyrusleeping/3983293
void RestoreTerminal(termios* initial_settings)
{
    tcsetattr(STDIN_FILENO, TCSANOW, initial_settings);
}

void SetTerminalToNonBlocking(termios* initial_settings)
{
    auto new_settings = termios();
    tcgetattr(STDIN_FILENO, initial_settings);

    new_settings = *initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 0;
    new_settings.c_cc[VTIME] = 0;

    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "keyboard_controller_node");
    auto n = ros::NodeHandle();

    auto rate = ros::Rate(30);

    if(argc != 4)
    {
        ROS_ERROR("usage: %s flightCommandTopicName fightControlTopicName odometryTopicName", argv[0]);
    }
    auto controller = DroneController(n, argv[1], argv[2], argv[3]);
    
    ROS_INFO(   "Hold down character. \n \
    Z for take_off, X for landing \n \
    M for motors on, N for motors off \n \
    r for up, f for down \n \
    w for forward, s for backward \n \
    a for yaw-left, d for yaw-right \n \
    q for rotate left, e for rotate right.\n\n");

    auto terminal_settings = termios();
    char buff[2] = {};
    SetTerminalToNonBlocking(&terminal_settings);

    ROS_INFO("Terminal set to input mode.");

    while(ros::ok())
    {
        auto length = read(STDIN_FILENO, buff, sizeof(buff));
        if(length > 0 )
        {
            char c = buff[0];
            switch (c)
            {
            case 'N':
                controller.sendControlCommand(CONTROL_COMMAND::MOTORS_OFF);
                break;
            case 'M':
                controller.sendControlCommand(CONTROL_COMMAND::MOTORS_ON);
                break;

            case 'Z':
                controller.sendControlCommand(CONTROL_COMMAND::TAKEOFF_START);
                break;
            case 'X':
                controller.sendControlCommand(CONTROL_COMMAND::LANDING_START);
                break;
            
            case '.':
                ROS_INFO("Exit character received. Stopping...");
                ros::shutdown(); // sets ros::ok() to false.
            break;
                default:
                //if( controller.getState() == CONTROLLER_STATE::ACTIVE ||
                //    controller.getState() == CONTROLLER_STATE::FLYING )
                {
                    switch (c)
                    {     
                        case 'r':
                            controller.sendFlightCommand(FLIGHT_COMMAND::UPWARDS);
                            //controller.sendControl(0,0,2,0);
                            break;
                        case 'f':
                            controller.sendFlightCommand(FLIGHT_COMMAND::DOWNWARDS);
                            //controller.sendControl(0,0,-2,0);
                            break;
                        case 'w':
                            controller.sendFlightCommand(FLIGHT_COMMAND::FORWARDS);
                            //controller.sendControl(1,0,0,0);
                            break;
                        case 's':
                            controller.sendFlightCommand(FLIGHT_COMMAND::BACKWARDS);
                            //controller.sendControl(-1,0,0,0);
                            break;
                        case 'a':
                            controller.sendFlightCommand(FLIGHT_COMMAND::YAW_LEFT);
                            //controller.sendControl(0,1,0,0);
                            break;
                        case 'd':
                            controller.sendFlightCommand(FLIGHT_COMMAND::YAW_RIGHT);
                            //controller.sendControl(0,-1,0,0);
                            break;
                        case 'q':
                            controller.sendFlightCommand(FLIGHT_COMMAND::ROT_LEFT);
                            //controller.sendControl(0,0,0,30);
                            break;
                        case 'e':
                            controller.sendFlightCommand(FLIGHT_COMMAND::ROT_RIGHT);
                            //controller.sendControl(0,0,0,-30);
                            break;
                        default:
                            ROS_WARN("Unknown input character: %c", c);
                    }
                }
            }
        }
        controller.publishLatestSpeed();
        ros::spinOnce();
        rate.sleep();
    }

    RestoreTerminal(&terminal_settings);

    return EXIT_SUCCESS;
}
