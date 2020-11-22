#ifndef DNNDETECTOR_HPP
#define DNNDETECTOR_HPP

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/dnn.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

#include <iostream>
#include <vector>

#include "image_geometry.h"

// test

/**
 * @brief Class to detect and track faces using openCV's DNN functions.
 * 
 */
class DNNDetector {
    
    private:
    cv::dnn::Net net;
    std::string inputLayerName, outputLayerName;
    
    /**
     * @brief Sets the image at the inputlayer of the Neural network.
     * Then runs the network over the image and takes the data at the output layer.
     * 
     * @param image 
     * @return cv::Mat 
     */
    cv::Mat detect(const cv::Mat& inputImage)
    {
        cv::Mat blob;
        cv::dnn::blobFromImage(inputImage, blob);

        net.setInput( blob, inputLayerName );
        return net.forward( outputLayerName );
    }

    public:

    /**
     * @brief Constructor for a new opencv::dnn::Net object
     * 
     * @param modelfile An openCV dnn compatible model file. (e.g. Caffe, Tensorflow)
     * @param configfile An openCV dnn compatible config file. Specifying the layer structure information contained in the model.
     * @param input_name Name of the input layer. Specified in the config file.
     * @param output_name Name of the output layer. Specified in the config file.
     */
    DNNDetector(  const std::string& modelfile
                , const std::string& configfile
                , std::string input_name = "data"
                , std::string output_name = "detection_out" )
    : net( cv::dnn::readNet(modelfile, configfile) )
    , inputLayerName( input_name )
    , outputLayerName( output_name )
    { 

    }

    /**
     * @brief Extracts the detected faces from the cv::Mat object returned by the "detect" function. 
     * It groups them in the returned vector without sorting. 
     * 
     * @param inputImage 
     * @return std::vector<cv::Rect> Bounding boxes of detected faces.
     */
    std::vector<cv::Rect> getDetectedFaces(const cv::Mat& inputImage, float tolerance = 0.7 )
    {
        cv::Mat output = detect(inputImage);
        std::vector<cv::Rect> boundingBoxes;

        if (!output.empty())
        {
            cv::Mat outputData(output.size[2], output.size[3], CV_32F, output.ptr<float>());

            for (int i = 0; i < outputData.rows; i++)
            {
                float confidence = outputData.at<float>(i, 2);
                //std::cout << confidence << std::endl;

                if (confidence > tolerance)
                {
                    int x1 = static_cast<int>(outputData.at<float>(i, 3) * inputImage.cols);
                    int y1 = static_cast<int>(outputData.at<float>(i, 4) * inputImage.rows);
                    int x2 = static_cast<int>(outputData.at<float>(i, 5) * inputImage.cols);
                    int y2 = static_cast<int>(outputData.at<float>(i, 6) * inputImage.rows);

                    boundingBoxes.push_back( cv::Rect( cv::Point(x1, y1), cv::Point(x2, y2) ) );
                }
            }
        }

        return boundingBoxes;
    }


    /**
     * @brief Function that takes an input image and overlays the faces the algorithm detects on it.
     * You can also request the detected faces as bounding boxes directly through an overload. 
     * 
     * @param inputImage 
     * @param color white by default.
     */
    void overlayDetectedFaces( cv::Mat& inputImage, std::vector<cv::Rect> faces, cv::Scalar color = cv::Scalar(0,255,0) )
    {
        for(const auto& face : faces)
        {
            cv::rectangle(inputImage, face, color, 4 );

            cv::Point distances = geo::getoffsets( {face.x + (face.width / 2), face.y + (face.height / 2 )}, {inputImage.cols, inputImage.rows} );
            cv::line(inputImage , { inputImage.cols / 2, inputImage.rows / 2 }
                                , { inputImage.cols / 2 + distances.x , inputImage.rows / 2}
                                , {0, 255, 255}, 2);

            cv::line(inputImage , {inputImage.cols / 2 + distances.x , inputImage.rows / 2}
                                , {inputImage.cols / 2 + distances.x , inputImage.rows / 2 + distances.y }
                                , {255, 255, 0}, 2 );
        }
    }

    void overlayDetectedFaces( cv::Mat& inputImage, double tolerance = 0.7, cv::Scalar color = cv::Scalar(0,255,0) )
    {
        overlayDetectedFaces(inputImage, getDetectedFaces(inputImage, tolerance), color );
    }

};

#endif