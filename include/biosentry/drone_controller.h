#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include "biosentry/AircraftFlightActions.h"

/**
 * @brief enum corresponding to all FlightActions the DJI drone API can provide.
 * These are:
 * enum class FlightActions()
 *{
 *    TurnMotorsOn,
 *    TurnMotorsOff,
 *    SetUrgentStopModeEnabled,
 *    SetUrgentStopModeDisabled,
 *    SetESCBeepEnabled,
 *    SetESCBeepDisabled,
 *    StartTakeoff,
 *    StartPrecisionTakeoff,
 *    CancelTakeoff,
 *    StartLanding,
 *    CancelLanding,
 *    ConfirmLanding,
 *    Reboot,
 *
 *    // Home actions also included here.
 *    StartGoHome,
 *    CancelGoHome,
 *    SetHomeLocationUsingCurrentAircraftLocation;
 * 
 */
enum class CONTROL_COMMAND {
    MOTORS_ON,
    MOTORS_OFF,
    URGENT_STOP_ENABLE,
    URGENT_STOP_DISABLE,
    ESC_BEEP_ENABLE,
    ESC_BEEP_DISABLE,
    TAKEOFF_START,
    TAKEOFF_PRECISION_START,
    TAKEOFF_CANCEL,
    LANDING_START,
    LANDING_CANCEL,
    LANDING_CONFIRM,
    REBOOT,

    GO_HOME_START,
    GO_HOME_CANCEL,
    GO_HOME_SET_CURRENT_LOCATION
};

enum class FLIGHT_COMMAND {
    FORWARDS,
    BACKWARDS,
    UPWARDS,
    DOWNWARDS,
    YAW_RIGHT,
    YAW_LEFT,
    ROT_RIGHT,
    ROT_LEFT,

};

class DroneController
{
private:
    ros::Publisher command_publisher;
    ros::Publisher control_publisher;
    ros::Subscriber odometry_subscriber;

    const double p_factor = 0.5;
    const double linearIncrement = 0.05;
    const double angularIncrement = 30;
    
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        auto newTwist = msg->twist.twist;
        
        auto dx_l = goalTwist.linear.x - newTwist.linear.x;
        auto dy_l = goalTwist.linear.y - newTwist.linear.y;
        auto dz_l = goalTwist.linear.z - newTwist.linear.z;
        
        auto dz_r = goalTwist.angular.z - newTwist.angular.z;

        currTwist.linear.x += dx_l * p_factor;
        currTwist.linear.y += dy_l * p_factor;
        currTwist.linear.z += dz_l * p_factor;

        currTwist.angular.z += dz_r * p_factor;
    }

public:

    bool motorsOn = false;

    geometry_msgs::Twist goalTwist = geometry_msgs::Twist();
    geometry_msgs::Twist currTwist = geometry_msgs::Twist();

    DroneController( ros::NodeHandle nodeHandle
                   , std::string commandTopic
                   , std::string controlTopic 
                   , std::string odometryTopic )
    : command_publisher( nodeHandle.advertise<biosentry::AircraftFlightActions>(commandTopic,1) )
    , control_publisher( nodeHandle.advertise<geometry_msgs::Twist>(controlTopic, 1))
    , odometry_subscriber( nodeHandle.subscribe(odometryTopic,1, &DroneController::odometryCallback, this) )
    {
    }

    /**
     * @brief Writes velocity values and sends them to the flight controller in the correct fields.
     * 
     * @param forwards relative to the drone, velocity in    m/s
     * @param sideways relative to the drone, velocity in    m/s
     * @param upwards relative to the drone, velocity in     m/s
     * @param rotate about the vertical axis in              deg/s
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

    /**
     * @brief Send a control command through the provided enum.
     * numeric value of the enum corresponds to the correct value on the app side.
     * @param command 
     */
    void sendControlCommand(CONTROL_COMMAND command)
    {
        auto msg = biosentry::AircraftFlightActions();
        msg.flightActions = static_cast<int>(command);
        command_publisher.publish(msg);        
    }

    /**
     * @brief More abstract manner of conveying directional control. Used in concessive keyboard input.
     * 
     * @param command 
     */
    void sendFlightCommand(FLIGHT_COMMAND command)
    {
        switch (command)
        {
        case FLIGHT_COMMAND::FORWARDS:
            goalTwist.linear.x += linearIncrement;
            break;
        case FLIGHT_COMMAND::BACKWARDS:
            goalTwist.linear.x -= linearIncrement;
            break;
        case FLIGHT_COMMAND::UPWARDS:
            goalTwist.linear.z += linearIncrement;
            break;
        case FLIGHT_COMMAND::DOWNWARDS:
            goalTwist.linear.z -= linearIncrement;
            break;
        case FLIGHT_COMMAND::YAW_RIGHT:
            goalTwist.linear.y += linearIncrement;
            break;
        case FLIGHT_COMMAND::YAW_LEFT:
            goalTwist.linear.y -= linearIncrement;
            break;
        case FLIGHT_COMMAND::ROT_RIGHT:
            goalTwist.angular.z += angularIncrement;       
        default:
            break;
        }
    }

    void publishLatestSpeed()
    {
        control_publisher.publish(currTwist);
    }
};

#endif // DRONE_CONTROLLER_H