#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "../include/biosentry/image_publisher.h"

/**
 * @brief This program is for testing purposes. 
 * One can provide it a video file, which is read using opencv. And published on a ros topic.
 * 
 * @param argc == 2 
 * @param argv1 filepath to the video file.
 * @return int 
 */
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_publisher_node");
    ros::Time::init();
    
    if(argc != 2 )
    {
        std::cerr << "Please provide a filepath to a video file. \n";
        return -1;
    }
    std::string filePath = std::string(argv[1]);
    std::cout << "The provided filepath: " << filePath << '\n';

    ImagePublisher imgpub = ImagePublisher( filePath, 24 );

    while(ros::ok())
    {
        imgpub.publishFrame();
    }

    return 0;
}
