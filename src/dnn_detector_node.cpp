//*//
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16MultiArray.h"
#include "cv_bridge/cv_bridge.h"

#include <string>

#include "../include/biosentry/subscription_publisher.h"
#include "../include/biosentry/opencv_dnn_detector.h"
#include "../include/biosentry/image_geometry.h"


DNNDetector* detector;
float tolerance = 0.95;
cv::Size* newsize;

/**
 * @brief Callback function to set the resolution at which the neural network processes input images.
 * This will do an openCV resize on the image before feeding it into the NN input layer.
 * While no data is provided it will use the native resolution of the input image.
 * 
 * @param msg array that should only have 2 values. 
 * The first for the new width and second for the height after resizing.
 */
void setResolution(std_msgs::UInt16MultiArray msg)
{
    //Check if a valid array is given.
    if(msg.data.size() == 2)
    {
        newsize->width = msg.data.at(0);
        newsize->height = msg.data.at(1);
    }
}

/**
 * @brief Callback function to change the tolerance of the neural network.
 * valid range is between 0.0 (tolerate everything) to 1.0 (only tolerate 100% certain detections)
 * 
 * @param msg 
 */
void setTolerance(std_msgs::Float32 msg)
{
    tolerance = msg.data;
}


sensor_msgs::Image detectFaces( sensor_msgs::ImageConstPtr input )
{
    auto wrapper = cv_bridge::toCvShare(input, "bgr8");
    auto image = wrapper->image;

    if( newsize && !newsize->empty() )
    {
        cv::resize(image, image, *newsize);
    }

    std::vector<cv::Rect> faces = detector->getDetectedFaces(image, tolerance );

    detector->overlayDetectedFaces(image, faces );

    sensor_msgs::Image out;
    wrapper->toImageMsg(out);

    return out;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_detector_node");
    std::cout << std::string(argv[0]) << "\n";

    if(argc != 8 )
    {
        std::cerr << "Please provide a filepath to a model and config file.\n"; // 1, 2
        std::cerr << "Also provide the names of the input and output layers of the DNN. (specified in the config file.)\n";
        std::cerr << "Also provide an input and output topic for video and movement response respectively.\n";
        std::cerr << "Also provide a frequency at which the node is throttled. Provide 0 to run at input topic's frequency.\n";

        return -1;
    }

    auto model_file = std::string(argv[1]);
    auto config_file = std::string(argv[2]);

    auto input_layer_name = std::string(argv[3]);
    auto output_layer_name =  std::string(argv[4]);

    DNNDetector det = DNNDetector( model_file, config_file ,input_layer_name ,output_layer_name  );
    SubscriptionPublisher<sensor_msgs::ImageConstPtr, sensor_msgs::Image> subpub(argv[5], argv[6], detectFaces);

    detector = &det;

    ros::NodeHandle n;

    cv::Size startSize = cv::Size();
    newsize = &startSize;
 
    ros::Subscriber tolsub = n.subscribe("biosentry/detector_tolerance", 1,  setTolerance);
    ros::Subscriber sizesub = n.subscribe("biosentry/detector_resolution", 1, setResolution);

    // frequency 0 means respond as fast to callbacks as you can. 
    double frequency = std::stod( std::string( argv[7] ) );
    if(frequency == 0.0)
    {
        ros::spin();
    }

    //Sleep for the remainder of free time within the assigned frequency.
    ros::Rate rate( frequency );
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    

    return 0;
}
