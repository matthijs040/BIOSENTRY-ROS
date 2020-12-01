#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include <stdio.h>
#include "sensor_msgs/NavSatFix.h"

#include "../../../devel/include/biosentry/AircraftFlightActions.h"
#include "../include/biosentry/servoing_3d.h"

bool isWithin(double val, double setPoint, double range)
{
    return (val < setPoint + range && val > setPoint - range);
}

auto initialOdom = nav_msgs::Odometry();
auto latestOdom = nav_msgs::Odometry();
bool initialMessage = false;
void OdometryCallback(nav_msgs::Odometry msg)
{
    if(!initialMessage)
    {
        initialMessage = true;
        initialOdom = msg;
    }
    latestOdom = msg;
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "drone_control_test_node");
    auto nh = ros::NodeHandle();

    // Rate of control loop thread.
    // Maximum rate of transmission that the DJI flight controller can work with:
    // https://developer.dji.com/api-reference/android-api/Components/FlightController/DJIFlightController.html#djiflightcontroller_virtualstickcontrolmodecategory_sendvirtualstickflightcontroldata_inline
    auto rate = ros::Rate(25);

    auto odom_sub = nh.subscribe<nav_msgs::Odometry>("android/drone/odom", 1, OdometryCallback);

    auto spinner = ros::AsyncSpinner(1);
    spinner.start();

    auto cmd_pub = nh.advertise<biosentry::AircraftFlightActions>("android/drone/biosentry/AircraftFlightActions", 1);
    auto vel_pub = nh.advertise<geometry_msgs::Twist>("/android/drone/geometry_msgs/Twist", 1);

    // Wait for the first odometry.
    ros::topic::waitForMessage<nav_msgs::Odometry>("/android/drone/odom");

    //send turn motors on.
    auto cmd_msg = biosentry::AircraftFlightActions();
        cmd_msg.flightActions = 0;
    cmd_pub.publish(cmd_msg);
    
    auto init_pos = initialOdom.pose.pose.position;
    auto goal =  geometry_msgs::Point(init_pos);
    goal.z += 5;
    auto controller = Servoing(goal);
    
    while(ros::ok())
    {
        // Has an initial location. App / Drone is online and communicating to ROS.
        if(initialMessage && initialOdom.pose.pose.position.x != 0.0)
        {
            // If controller has reached goal.
            if(controller.angFinished && controller.linFinished)
            {
                // If it is the higher vertical goal.
                if(controller.goal.z == 3)
                {
                    goal.z -= 2;
                    controller.setGoal(goal);
                }
                else // If it is the lower goal.
                {
                    goal.z += 2;
                    controller.setGoal(goal);
                }
            }

            // Calculate and send new command.
            auto pub_msg = controller.updatePath(latestOdom);
            vel_pub.publish(pub_msg);
            std::cout << "lin: X: " << pub_msg.linear.x << " Y: " << pub_msg.linear.y << " Z: " << pub_msg.linear.z << " | ang Z: " << pub_msg.angular.z << "\n"; 
        }
        rate.sleep();
    }

    return 0;
}
