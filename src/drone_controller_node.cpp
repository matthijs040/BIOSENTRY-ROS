#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "drone_controller_node");

    // Rate of control loop thread.
    // Maximum rate of transmission that the DJI flight controller can work with:
    // https://developer.dji.com/api-reference/android-api/Components/FlightController/DJIFlightController.html#djiflightcontroller_virtualstickcontrolmodecategory_sendvirtualstickflightcontroldata_inline
    auto controlRate = ros::Rate(25);


    return 0;
}
