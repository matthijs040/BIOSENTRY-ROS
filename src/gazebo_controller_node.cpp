#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>

ros::Publisher gazebo_odom_publisher;
ros::Publisher gazebo_modelstate_publisher;
std::string model_name;
auto out_odom = nav_msgs::Odometry();

void TwistCallback(geometry_msgs::Twist in)
{
    //std::cout << "twist received \n";

    auto out_modelstate = gazebo_msgs::ModelState();

    out_modelstate.model_name = model_name;
    out_modelstate.pose = out_odom.pose.pose;
    out_modelstate.twist = in; //maybe need compensation in linear Z for gravity?

    

    gazebo_modelstate_publisher.publish(out_modelstate);
}

void LinkStateCallback(gazebo_msgs::ModelStates in)
{
    //for(const auto& name : in.name)
    for(unsigned long i = 0; i < in.name.size(); i++)
    {
        if(in.name.at(i) == model_name)
        {
            out_odom.pose.pose = in.pose.at(i);
            out_odom.twist.twist = in.twist.at(i);
            out_odom.child_frame_id = model_name;

            gazebo_odom_publisher.publish(out_odom);
        }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gazebo_controller_node");
    auto node_handle = ros::NodeHandle();

    auto get_modelstate_topic = std::string();
    auto get_twist_topic = std::string();

    auto set_linkstate_topic = std::string();
    auto odometry_topic = std::string();

    if(argc != 6)
    {
        ROS_ERROR("provide the required parameters.");
        ros::shutdown();
        return EXIT_FAILURE;
    }

    model_name = argv[1]; // Name of the link to read odom from and send control to.
    get_modelstate_topic = argv[2];
    get_twist_topic = argv[3];

    set_linkstate_topic = argv[4];
    odometry_topic = argv[5];

    ROS_INFO(
        "got the model name: %s \n\
         listening on: %s for modelstate from gazebo\n\
         listening on: %s for twist messages from a controller \n\
         publish modelstate with a new velocity on: %s \n\
         publish odometry for external controller on: %s ", model_name.c_str(), get_modelstate_topic.c_str(), get_twist_topic.c_str(), set_linkstate_topic.c_str(), odometry_topic.c_str());
    
    auto gazebo_model_subscriber     = node_handle.subscribe<gazebo_msgs::ModelStates>(get_modelstate_topic, 1, LinkStateCallback);
    auto gazebo_velocity_subscriber = node_handle.subscribe<geometry_msgs::Twist>(get_twist_topic, 1, TwistCallback);

    gazebo_odom_publisher           = node_handle.advertise<nav_msgs::Odometry>(odometry_topic, 1);
    gazebo_modelstate_publisher     = node_handle.advertise<gazebo_msgs::ModelState>(set_linkstate_topic, 1);

    auto rate = ros::Rate(60);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return EXIT_SUCCESS;
}
