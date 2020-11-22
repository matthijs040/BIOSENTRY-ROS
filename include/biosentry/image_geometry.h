#ifndef GEO_PROC_HPP
#define GEO_PROC_HPP

#include <vector>
#include <stdexcept>

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>


/**
 * @brief   Helper functions for doing transforms on geometric objects in images.
 */
namespace geo
{
    inline cv::Point mleft(const cv::Mat& img)
    { 
        return { 0, img.rows / 2 };
    }
    
    inline cv::Point mtop(const cv::Mat& img)
    {
        return { img.cols / 2, 0 };
    }

    inline cv::Point mright(const cv::Mat& img)
    {
        return { img.cols, img.rows / 2 };
    }
    
    inline cv::Point mbot(const cv::Mat& img)
    {
        return { img.cols / 2, img.rows };
    }

    inline cv::Point mmid(const cv::Mat& img)
    {
        return { img.cols / 2, img.rows / 2 };        
    }

    inline double rotationPerPixel(const int pixels, const double FOV)
    {
        return FOV / pixels;
    }

    inline cv::Point getoffsets(const cv::Point point, const cv::Size dims = { 1280 , 720 } )
    {
        return {    -( dims.width  / 2 ) + point.x,    
                (   -( dims.height / 2 ) + point.y ) } ;
    }

    inline std::vector<cv::Point> getoffsets(const std::vector<cv::Point>& in, const cv::Size dims = { 1280 , 720 } )
    {
        std::vector<cv::Point> out;
        out.reserve( in.size() );

        for(const auto& point : in ) 
        {
            out.emplace_back( getoffsets(point, dims) );
        }

        return out;
    }

    inline cv::Point getCentre(cv::Rect rect)
    {
        return { rect.x + ( rect.width / 2 ), rect.y + ( rect.height / 2 ) };
    }

} // geo


#endif // GEO_PROC_HPP