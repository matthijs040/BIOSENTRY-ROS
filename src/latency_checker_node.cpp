#include "ros/ros.h"
#include "geometry_msgs/Point.h"

#include <chrono>

bool running = true;

//params that can change depending on test on the client side.
using testType = geometry_msgs::Point;
std::string testTypeName = "/geometry_msgs/Point";
std::string testTopicName = "/android/phone/loopback";

void callback(testType)
{
    running = false;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "latency_checker_node");
    auto nh = ros::NodeHandle();

    auto pub = nh.advertise<testType>("/android/phone/loopback/geometry_msgs/Point", 1);
    auto sub = nh.subscribe<testType>("/android/phone/loopback", 1, callback);

    auto rate = ros::Rate(240);

    pub.publish(geometry_msgs::Point());

    // from: https://en.cppreference.com/w/cpp/chrono
    auto begin = std::chrono::steady_clock::now();

    auto spinner = ros::SingleThreadedSpinner();
    while(running)
    {
        ros::spinOnce();
        pub.publish(geometry_msgs::Point());
        rate.sleep();
    }

    auto end = std::chrono::steady_clock::now();

    std::chrono::duration<double> elapsed_seconds = end-begin;
    std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n"; 

    return 0;
}
