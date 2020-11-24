#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "stdio.h"


void AccelerometerCallback(geometry_msgs::Vector3::ConstPtr msg)
{

    //std::cout << "message is: " << msg << "\n";

    if(msg->z > .5)
    {
        std::cout << "Rising\n";
    }
    else if( msg->z < -.5)
        std::cout << "Falling\n";
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "drone_control_test_node");
    auto nh = ros::NodeHandle();


    // Rate of control loop thread.
    // Maximum rate of transmission that the DJI flight controller can work with:
    // https://developer.dji.com/api-reference/android-api/Components/FlightController/DJIFlightController.html#djiflightcontroller_virtualstickcontrolmodecategory_sendvirtualstickflightcontroldata_inline
    auto rate = ros::Rate(25);


    auto vel_sub = nh.subscribe<geometry_msgs::Vector3>("/android/drone/accelerometer", 1, AccelerometerCallback);

    auto spinner = ros::AsyncSpinner(1);
    spinner.start();

    while(ros::ok())
    {
        rate.sleep();
    }

    return 0;
}
