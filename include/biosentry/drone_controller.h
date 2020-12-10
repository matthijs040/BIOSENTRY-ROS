#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#include <memory>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#include "biosentry/AircraftFlightActions.h"
#include "math_utils.h"

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
    MOTORS_ON, // Supported by flight controller firmware 3.1.0.0 or above.
    MOTORS_OFF,
    URGENT_STOP_ENABLE,
    URGENT_STOP_DISABLE,
    ESC_BEEP_ENABLE,
    ESC_BEEP_DISABLE,
    TAKEOFF_START, // If the motors are already on, this command cannot be executed
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

enum class CONTROLLER_STATE {
    IDLE,       // Drone is landed and motors are off.
    ACTIVE,     // Drone is landed but motors are on.
    TAKING_OFF, // Takeoff is considered complete when the aircraft is hovering 1.2 meters (4 feet) above the ground. 
    FLYING,  
    LANDING,
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
    ros::NodeHandle node_handle;
    ros::Publisher command_publisher;
    ros::Publisher control_publisher;
    ros::Subscriber odometry_subscriber;

    ros::Timer state_checker;

    nav_msgs::Odometry latest_odom = nav_msgs::Odometry();
    CONTROLLER_STATE current_state = CONTROLLER_STATE::IDLE;

    // Proportional factor value on the velocity difference.
    const double p_factor = 0.5;

    // Steps of velocity change per keyboard button detection.
    const double linearIncrement = 0.05;
    const double angularIncrement = (2 * M_PI / 32);

    // Maximum velocities of the DJI drone virtual stick controls:
    // https://developer.dji.com/api-reference/android-api/Components/FlightController/DJIFlightController.html#djiflightcontroller_djivirtualstickrollpitchcontrolmode_inline
    const double angularMaximum = (2* M_PI / 4);
    const double linearMaximum = 3;

    bool canSendManualCommands = true;

    /**
     * @brief Callback that handles transition between intelligent states of the drone (LANDING, TAKING_OFF)
     * And manual states of control (ACTIVE)
     * 
     * Note that when the drone is landed and the motors are on it can take off with FLIGHT_COMMANDs. 
     * Skipping the intelligent takeoff and landing routines.
     */
    void controllerStateCallback(const ros::TimerEvent&)
    {
        if(current_state == CONTROLLER_STATE::LANDING)
        {
            // If the drone has landed.
            if(latest_odom.pose.pose.position.z == 0)
                current_state = CONTROLLER_STATE::IDLE;
            state_checker.stop();
        }
        if(current_state == CONTROLLER_STATE::TAKING_OFF)
        {
            if(latest_odom.pose.pose.position.z >= 1.1 )
                current_state = CONTROLLER_STATE::FLYING;
            state_checker.stop();
        }
    }

    double throttle(double& val, double& oldVal)
    {
        val = oldVal;
        return 0;
    }
    
    /**
     * @brief Callback that gives the controller recent odometry information.
     * This is used in combination with a desired velocity set by external input 
     * to calculate a speed to send to the drone to make it accelerate gradually.
     * @param msg 
     */
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        auto newTwist = msg->twist.twist;
        
        auto dx_l = goalTwist.linear.x - newTwist.linear.x;

        dx_l = ( ( fabs( ( currTwist.linear.x + (dx_l * p_factor) ) ) > linearMaximum) ? 
                    throttle(goalTwist.linear.x, newTwist.linear.x) : 
                    dx_l );

        auto dy_l = goalTwist.linear.y - newTwist.linear.y;
        if(fabs( ( currTwist.linear.y + (dy_l * p_factor) ) ) > linearMaximum)
        {
            ROS_WARN("hit Y max. Throttling");
            dy_l = 0;
            goalTwist.linear.y = newTwist.linear.y;
        }


        dy_l = ( ( fabs( ( currTwist.linear.y + (dy_l * p_factor) ) ) > linearMaximum) ? 0 : dy_l );

        auto dz_l = goalTwist.linear.z - newTwist.linear.z;
        if(fabs( ( currTwist.linear.z + (dz_l * p_factor) ) ) > linearMaximum )
            ROS_WARN("hit Z max. Throttling");

        dz_l = ( ( fabs( ( currTwist.linear.z + (dz_l * p_factor) ) )  > linearMaximum) ? 0 : dz_l);

        // Calculate angular velocity difference and threshold if too high.
        auto dz_r = goalTwist.angular.z - newTwist.angular.z;
        if( ( fabs( ( currTwist.angular.z + (dz_r * p_factor) ) ) > angularMaximum) > angularMaximum)
            ROS_WARN("hit angular max. throttling.");

        dz_r = ( ( ( fabs(  currTwist.angular.z + (dz_r * p_factor)  ) > angularMaximum ) ) ? 0 : dz_r );

        currTwist.linear.x += dx_l * p_factor;
        currTwist.linear.y += dy_l * p_factor;
        currTwist.linear.z += dz_l * p_factor;

        currTwist.angular.z += dz_r * p_factor;
    
        ROS_INFO("latest twist is: %f %f %f %f\n", currTwist.linear.x, currTwist.linear.y , currTwist.linear.z, currTwist.angular.z );
    }

public:

    geometry_msgs::Twist goalTwist = geometry_msgs::Twist();
    geometry_msgs::Twist currTwist = geometry_msgs::Twist();

    DroneController( ros::NodeHandle nodeHandle
                   , std::string commandTopic
                   , std::string controlTopic 
                   , std::string odometryTopic )
                   : node_handle(nodeHandle)
                   , command_publisher(     node_handle.advertise<biosentry::AircraftFlightActions>(commandTopic,1) )
                   , control_publisher(     node_handle.advertise<geometry_msgs::Twist>(controlTopic, 1))
                   , odometry_subscriber(   node_handle.subscribe(odometryTopic,1, &DroneController::odometryCallback, this) )
                   , state_checker(         node_handle.createTimer(ros::Duration(0.5), &DroneController::controllerStateCallback, this) )
    {
    }

    const CONTROLLER_STATE& getState()
    {
        return current_state;
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
        // std::cout << "received the command enum: " << static_cast<typename std::underlying_type<CONTROL_COMMAND>::type>(command) << "\n";
        // std::cout << "current state: " << static_cast<typename std::underlying_type<CONTROLLER_STATE>::type>(current_state) << '\n';
        if( command == CONTROL_COMMAND::TAKEOFF_START || 
            command == CONTROL_COMMAND::TAKEOFF_PRECISION_START )
        {
            current_state = CONTROLLER_STATE::TAKING_OFF;
            state_checker.start();
        }
        if( command == CONTROL_COMMAND::LANDING_START )
        {
            current_state = CONTROLLER_STATE::LANDING;
            state_checker.start();
        }
        if( command == CONTROL_COMMAND::MOTORS_ON )
        {
            if(current_state == CONTROLLER_STATE::IDLE)
            {
                current_state = CONTROLLER_STATE::ACTIVE;
                ROS_INFO("current state set to active");
            }

        }

        auto msg = biosentry::AircraftFlightActions();
        msg.flightActions = static_cast<int>(command);
        command_publisher.publish(msg);        
    }

    /**
     * @brief More abstract manner of conveying directional control. Used in concessive keyboard input.
     * Checks if the aircraft is in a state where it is allowed to move.
     * @param command 
     */
    void sendFlightCommand(FLIGHT_COMMAND command)
    {
        if( current_state != CONTROLLER_STATE::ACTIVE && 
            current_state != CONTROLLER_STATE::FLYING )
        {
            ROS_WARN("denied flight command as drone is not active.");
            return;
        }

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
            break;
        case FLIGHT_COMMAND::ROT_LEFT:
            goalTwist.angular.z -= angularIncrement;
            break;
        default:
            break;
        }
    }

    void publishLatestSpeed()
    {
        if( current_state == CONTROLLER_STATE::ACTIVE ||
            current_state == CONTROLLER_STATE::FLYING)
        { control_publisher.publish(currTwist); }
    }
};

#endif // DRONE_CONTROLLER_H