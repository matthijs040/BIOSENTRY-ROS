#include "ros/ros.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

int main(int argc, char *argv[])
{
    if(argc == 2 )
    {
        ros::init(argc, argv, "rtmp_recv_node");
        auto n = ros::NodeHandle();
        std::string URL = "";
        URL = std::string(argv[1]);

        ROS_INFO("Got the URL: %s", URL.c_str() );
        std::string protocol = URL.substr(0, URL.find("://"));

        // Got an RTMP URL. Attempt to parse.
        if(protocol == "rtmp")
        {
            ros::Publisher pub = n.advertise<sensor_msgs::Image>("biosentry/image_raw", 1, true);
            cv::VideoCapture cap = cv::VideoCapture(URL);

            cv::Mat frame = cv::Mat();
            auto imsg = cv_bridge::CvImage();
            imsg.encoding = "bgr8";
            
            while(cap.read(frame))
            {
                imsg.image = frame;
                pub.publish(imsg.toImageMsg());
            }
        }
    }

    return 0;
}
