#ifndef SERVOING_H
#define SERVOING_H

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#include "ros/console.h"
#include "angles/angles.h"
#include "tf/tf.h"

class Servoing 
{
    private:
    float angularP = 1.2;
    float linearP = 1.2;

    bool isWithin(double val, double setPoint, double range)
    {
        return (val < setPoint + range && val > setPoint - range);
    }
    


    public:
    geometry_msgs::Point goal;
    bool angFinished, linFinished;

    // Create a Servoing object that, when polled for a new command with updated telemetry will provide the right command to reach the destination.
    // The end condition of which would be a Twist message with no motion.
    Servoing(geometry_msgs::Point goal)
    {
        this->goal = goal;
        angFinished = false;
        linFinished = false;
    }

    // Override the goal with a new goal position + orientation.
    void setGoal(geometry_msgs::Point goal)
    {
        this->goal = goal;
        angFinished = false;
        linFinished = false;
    }

    void setGoal(geometry_msgs::PoseStamped goal)
    {
        setGoal(goal.pose.position);
    }

    geometry_msgs::Twist updatePath(const geometry_msgs::Point position, const double currentAngle)
    {
        geometry_msgs::Twist move = geometry_msgs::Twist();

        double desiredAngle = atan2( goal.y - position.y ,  goal.x - position.x  );
        if( currentAngle < desiredAngle + 0.01 && currentAngle > desiredAngle  - 0.01 )
        {
            angFinished = true;
            ROS_DEBUG("rotation finished.");
        }
        else //NOTE: This check has to be done to correct the angle again after long linear motion to correct for drift.
        {
            angFinished = false;
        }

        if(!angFinished)
        {
            double deviation =  desiredAngle - currentAngle;

            // NOTE: Theshold to correct for huge deviations when passing the PI -PI angle on the unity circle.
            if( deviation > M_PI) 
            {
                    deviation -= (2 * M_PI);
            }
            else if( deviation < -M_PI)
            {
                    deviation += (2 * M_PI);   
            }

            move.angular.z = angularP *  deviation ;
            // TODO: Threshold movement?

        }
        

        if(!linFinished)
        {
             const double cX = position.x,  cY = position.y,  cZ = position.z,
                          gX = goal.x,      gY = goal.y,      gZ = goal.z;

            if(  isWithin(cX, gX, 0.01)
              && isWithin(cY, gY, 0.01)
              && isWithin(cZ, gZ, 0.01) )
            {
                linFinished = true;
                ROS_DEBUG("linear motion finished.");
            }
            else
            {
                const double deviationX = gX - cX;
                const double deviationY = gY - cY;
                const double deviationZ = gZ - cZ;
                move.linear.x = ( std::abs(linearP * deviationX) + std::abs(linearP * deviationY) ) / 2 ;
                move.linear.z = ( std::abs(linearP * deviationZ) );

                // TODO: Threshold movement?

            }
        }

        return move;
    }


    // Calculates the point-shoot twist required to make a robot move towards goal.
    // This calculation requires the most recent odom update.
    geometry_msgs::Twist updatePath(const nav_msgs::Odometry& odomUpdate)
    {
        geometry_msgs::Twist move = updatePath(odomUpdate.pose.pose.position, tf::getYaw(odomUpdate.pose.pose.orientation) );
        return move;
       
    }

}; // class Servoing

//Servoing_H
#endif