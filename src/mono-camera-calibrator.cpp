#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include "mono-camera-calibrator.hpp"

using namespace std;
using namespace cv;
using namespace stereo_pi::mono;

double MonoCameraCalibrator::computeReprojectionErrors(const vector< Mat >& rvecs, const vector< Mat >& tvecs,
                                 const Mat& cameraMatrix , const Mat& distCoeffs) {
  vector< Point2f > imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  vector< float > perViewErrors;
  perViewErrors.resize(object_points_.size());

  for (i = 0; i < (int)object_points_.size(); ++i) {
    projectPoints(Mat(object_points_[i]), rvecs[i], tvecs[i], cameraMatrix,
                  distCoeffs, imagePoints2);
    err = norm(Mat(image_points_[i]), Mat(imagePoints2), NORM_L2);
    int n = (int)object_points_[i].size();
    perViewErrors[i] = (float) std::sqrt(err*err/n);
    totalErr += err*err;
    totalPoints += n;
  }
  return std::sqrt(totalErr/totalPoints);
}

void MonoCameraCalibrator::add_chessboard_sample(cv::Mat& image, bool verbose=false)
{
    bool found = false;
    Size board_size = Size(board_width_, board_height_);
    found = cv::findChessboardCorners(image, board_size, corners_,
                                      CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
    if (found)
    {
      cornerSubPix(image, corners_, cv::Size(5, 5), cv::Size(-1, -1),
                   TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));
      if(verbose)
      {
        Mat colored_mat;
        cvtColor(image, colored_mat, COLOR_GRAY2BGR);
        drawChessboardCorners(colored_mat, board_size, corners_, found);
        imshow("Extracted corners", colored_mat);
        waitKey(1);
      }
    }
    
    vector< Point3f > obj;
    for (int i = 0; i < board_height_; i++)
      for (int j = 0; j < board_width_; j++)
        obj.push_back(Point3f((float)j * square_size_, (float)i * square_size_, 0));

    if (found) {
      cout << k_ << ". Found corners!" << endl;
      image_points_.push_back(corners_);
      object_points_.push_back(obj);
      k_++;
    }

    for (int i = 0; i < image_points_.size(); i++) 
    {
      vector< Point2f > v1;
      for (int j = 0; j < image_points_[i].size(); j++) 
      {
        v1.push_back(Point2f((double)image_points_[i][j].x, (double)image_points_[i][j].y));
        
      }
      _img_points_.push_back(v1);
    }
    destroyAllWindows();
}

MonoCameraCalibrator::MonoCameraCalibrator(int board_width, 
                                           int board_height, 
                                           float square_size):k_(0), 
                                                              board_width_(board_width),
                                                              board_height_(board_height),
                                                              square_size_(square_size)
{
    
}

bool MonoCameraCalibrator::process(int height, int weight, std::vector< cv::Mat >& rvecs, std::vector< cv::Mat >& tvecs,
                                cv::Mat& cameraMatrix , cv::Mat& distCoeffs)
{
  if(object_points_.size() > 0)
  {
    int flag = 0;
    flag |= CALIB_FIX_K5;
    cv::Size resolution = cv::Size(weight, height);
    calibrateCamera(object_points_, image_points_, resolution, cameraMatrix, distCoeffs, rvecs, tvecs, flag);
    return true;
  }
  else
  {
    printf("[ERROR] Couldn't find any samples\n");
    return false;
  }
  
}

void MonoCameraCalibrator::save_results_to_yaml(const std::string& filename, const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs)
{
  
}

// void MonoCameraCalibrator::save_result_to_cv_config(const std::string& filename, cv::Mat& cameraMatrix , const cv::Mat& distCoeffs)
// {
//   FileStorage fs(filename, FileStorage::WRITE);
//   fs << "K" << cameraMatrix;
//   fs << "D" << distCoeffs;
//   fs << "board_width" << board_width_;
//   fs << "board_height" << board_height_;
//   fs << "square_size" << square_size_;
// }



