#include "ros/ros.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

#include "opencv2/dnn/dnn.hpp"  

void image_callback(sensor_msgs::ImageConstPtr msg)
{
    auto bridge_image = cv_bridge::toCvShare(msg);
    
}

int main(int argc, char *argv[])
{
    if(argc == 2 )
    {
        ros::init(argc, argv, "face_detec_node");
        auto n = ros::NodeHandle();
        std::string topic = "";
        topic = std::string(argv[1]);

        ROS_INFO("Got the image topic: %s", topic.c_str() );
        auto rate = ros::Rate(60);

        auto subscriber = n.subscribe<sensor_msgs::Image>(topic, 0, image_callback);

   

        ros::spin();

    
    


    }

    ROS_ERROR("Got no image topic to subscribe to. Stopping.");
    return 0;
}
