#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include "biosentry/AircraftFlightActions.h"

class DroneController
{
private:
    ros::Publisher command_publisher;
    ros::Publisher control_publisher;
    ros::Subscriber odometry_subscriber;

    void odometryCallback(const nav_msgs::Odometry::ConstPtr&)
    {
        // Will be used to ease into the final velocity based on current odometry using P-control.
    }

public:
    DroneController( ros::NodeHandle nodeHandle
                   , std::string commandTopic
                   , std::string controlTopic 
                   , std::string odometryTopic )
    : command_publisher( nodeHandle.advertise<biosentry::AircraftFlightActions>(commandTopic,1) )
    , control_publisher( nodeHandle.advertise<geometry_msgs::Twist>(controlTopic, 1))
    , odometry_subscriber( nodeHandle.subscribe(odometryTopic,1, &DroneController::odometryCallback, this) )
    {
    }

    void SendEnableMotors()
    {
        auto msg = biosentry::AircraftFlightActions(); msg.flightActions = 0;
        command_publisher.publish( msg );
    }

    void SendDisableMotors()
    {
        auto msg = biosentry::AircraftFlightActions(); msg.flightActions = 1;
        command_publisher.publish( msg );
    }

    /**
     * @brief Writes velocity values and sends them to the flight controller in the correct fields.
     * 
     * @param forwards relative to the drone, velocity in    m/s
     * @param sideways relative to the drone, velocity in    m/s
     * @param upwards relative to the drone, velocity in     m/s
     * @param rotate about the vertical axis in             deg/s
     */
    void sendControl(double forwards, double sideways, double upwards, double rotate)
    {
        auto msg = geometry_msgs::Twist();

        msg.linear.x = forwards;
        msg.linear.y = sideways;
        msg.linear.z = upwards;
        msg.angular.z = rotate;

        control_publisher.publish(msg);
    }
};

#endif // DRONE_CONTROLLER_H