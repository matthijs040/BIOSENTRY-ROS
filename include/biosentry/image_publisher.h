#include <ros/ros.h>

#include <std_msgs/Int16.h>

#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>

#include "cv_bridge/cv_bridge.h"

class ImagePublisher
{
    private:

    // Objects whose scope expose core ros functionality needed here
    ros::NodeHandle nodehandle;
    ros::Subscriber delaysub;
    std::string filepath;
    
    ros::Publisher publisher;

     //= it.advertise("out_image_base_topic", 1);
    cv::VideoCapture videoCapture;

    int16_t framerate;


    /**
     * @brief A function to validate a filepath entering class scope.
     * It is checked if an openCV video stream can be opened to the provided string as file.
     * If this fails it will throw an invalid argument exception.
     * 
     * @param filepath 
     * @throw std::invalid_argument("filepath")
     * @return std::string 
     */
    std::string validateFilePath(std::string filepath)
    {
        // A little inefficient but non-critical check. As it is only done once per file.
        // Is also validates if openCV can open / work with it.
        if( filepath.empty() )
        { 
            std::cout << "The filepath: " << filepath << " is empty. Using default video input device as fallback. \n";
        }
        return filepath;
    }

    void setSpeed( const std_msgs::Int16::ConstPtr& framerate)
    {
        this->framerate = int16_t(framerate->data);
        std::cout << "framerate of the video set to: " << framerate << '\n';
    }

    public:

    /**
     * @brief Construct a new Image Publisher object
     * If the filepath is empty it uses 0 (default input device / webcam ) instead.
     * @param filepath 
     * @param topicName 
     */
    ImagePublisher(std::string filepath, int16_t framerate, std::string topicname = "/biosentry/image_raw" )
    : nodehandle( ros::NodeHandle() )
    , delaysub( nodehandle.subscribe("biosentry/video_speed", 1, &ImagePublisher::setSpeed, this ) )
    , filepath( validateFilePath(filepath) )
    , publisher( nodehandle.advertise<sensor_msgs::Image>( topicname , 1 ) )
    , videoCapture( this->filepath )
    , framerate( framerate )
    { }

    /**
     * @brief Sets the filepath to a new videofile.
     * The path is validated to point to an openable video file.
     * If this fails an invalid argument exception is thrown.
     * 
     * @param newFilePath 
     */
    void setFilePath(std::string filepath)
    {
        this->filepath = validateFilePath(filepath);
    }

    inline void opencap()
    {
        if( filepath.empty() )
        {
            videoCapture.open(0);
        }
        else
        {
            videoCapture.open(filepath);
        }
    }

    /**
     * @brief This function publishes a single frame of the video file provided on construction.
     * The idea is that this allows the user to set the pace of image publishings.  
     * The function will publish the first frame again if it gets past the video's end.
     */
    void publishFrame()
    {
        ros::spinOnce();
        
        if( !videoCapture.isOpened() )
        {
            opencap();
        }

        cv::Mat frame; 
        if(videoCapture.read(frame))
        {
            publisher.publish( cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg() );

            if(framerate != 0) // If a framerate limit is given:
            {
                usleep( (1 * 1000000) / framerate );
            }

        }
        else
        {
            if( filepath.empty() )
            {
                videoCapture.open(0);
            }
            else
            {
                opencap();
            }
        }
    }

    /**
     * @brief This function publishes the entire provided video file.
     * Calling it again will replay the video.
     */
    void publishVideo()
    {
        opencap();

        cv::Mat frame;

        while( videoCapture.read(frame) )
        {
            publisher.publish( cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg() );

            if(framerate != 0) // If a framerate limit is given:
            {
                usleep( (1 * 1000000) / framerate );
            }

            ros::spinOnce();
        }
    }

};
