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

    auto rate = ros::Rate(60);

    if(argc != 4)
    {
        ROS_ERROR("usage: %s flightCommandTopicName fightControlTopicName odometryTopicName", argv[0]);
    }
    auto controller = DroneController(n, argv[1], argv[2], argv[3]);
    
    ROS_INFO(   "Hold down character. \n \
    Z for motors on, X for motors off \n \
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
            auto c = buff[0];
            //std::cout << "character is: " << c << '\n';
            switch (c)
            {
            case 'Z':
                controller.SendEnableMotors();
                break;
            case 'X':
                controller.SendDisableMotors();
                break;
            case 'r':
                controller.sendControl(0,0,1,0);
                break;
            case 'f':
                controller.sendControl(0,0,-1,0);
                break;
            case 'w':
                controller.sendControl(1,0,0,0);
                break;
            case 's':
                controller.sendControl(-1,0,0,0);
                break;
            case 'a':
                controller.sendControl(0,1,0,0);
                break;
            case 'd':
                controller.sendControl(0,-1,0,0);
                break;
            case 'q':
                controller.sendControl(0,0,0,30);
                break;
            case 'e':
                controller.sendControl(0,0,0,-30);
                break;
            case '.':
                ROS_INFO("Exit character received. Stopping...");
                ros::shutdown();
                break;
            default:
                ROS_WARN("Unknown input character: %c", c);
                break;
            }
        }

        rate.sleep();
    }

    RestoreTerminal(&terminal_settings);

    return EXIT_SUCCESS;
}
