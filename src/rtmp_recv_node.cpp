#include "ros/ros.h"

#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/highgui/highgui.hpp"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rtmp_recv_node");
    auto n = ros::NodeHandle();
    std::string URL = "";
    n.getParam("url", URL );

    ROS_INFO("Got the URL: %s", URL.c_str() );
    
    // Got an RTMP URL. Attempt to parse.
    if(URL.substr(0,3) == "rtmp://")
    {
        cv::VideoCapture cap = cv::VideoCapture(URL);
        cv::Mat frame = cv::Mat();
        while(cap.read(frame))
        {
            std::cout << "I have a frame of width: " << frame.cols << " height: " << frame.rows << "\n";
        }
    }

    return 0;
}
