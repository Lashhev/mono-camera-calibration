#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
// #define CALIB_LOG(msg, ...) printf("[ERROR] %s \n ", msg, __VA_ARGS__)
namespace stereo_pi
{
    namespace mono 
    {
        class MonoCameraCalibrator
        {
           public:
            MonoCameraCalibrator(int board_width, int board_height, float square_size);
            void add_chessboard_sample(cv::Mat& image);
            bool process(int height, int weight, std::vector< cv::Mat >& rvecs, std::vector< cv::Mat >& tvecs,
                                cv::Mat& cameraMatrix , cv::Mat& distCoeffs);
            double computeReprojectionErrors(const std::vector< cv::Mat >& rvecs, const std::vector< cv::Mat >& tvecs,
                                 const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs);
            std::vector< std::vector< cv::Point2f > > get_img_points()
            {
                return _img_points_;
            }
            std::vector< std::vector< cv::Point3f > > get_object_points()
            {
                return object_points_;
            }
           private:
            std::vector< std::vector< cv::Point3f > > object_points_;
            std::vector< std::vector< cv::Point2f > > image_points_;
            std::vector< cv::Point2f > corners_;
            std::vector< std::vector< cv::Point2f > > _img_points_; 
            int board_width_;
            int board_height_;
            float square_size_;
            size_t k_;
        };
    }
}
