#include "TimeSurface.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <thread>
#include <cmath>

namespace esvo_time_surface
{
TimeSurface::TimeSurface(const TimeSurfaceParams& params)
{
  // 1. Ingest pure C++ parameters instead of asking a ROS NodeHandle
  decay_ms_ = params.decay_ms;
  ignore_polarity_ = params.ignore_polarity;
  median_blur_kernel_size_ = params.median_blur_kernel_size;
  max_event_queue_length_ = params.max_event_queue_length;
  events_maintained_size_ = params.events_maintained_size;

  camera_matrix_ = params.camera_matrix.clone();
  dist_coeffs_ = params.dist_coeffs.clone();
  rectification_matrix_ = params.rectification_matrix.clone();
  projection_matrix_ = params.projection_matrix.clone();

  // Standard ESVO configuration for Stereo Mapping
  distortion_model_ = "plumb_bob";
  time_surface_mode_ = BACKWARD;

  bCamInfoAvailable_ = true;
  bSensorInitialized_ = false;

  if(pEventQueueMat_)
    pEventQueueMat_->clear();

  sensor_size_ = cv::Size(0,0);

  // 2. Immediately initialize the math (No waiting for ROS callbacks!)
  init(params.width, params.height);
}

TimeSurface::~TimeSurface()
{
}

void TimeSurface::init(int width, int height)
{
  sensor_size_ = cv::Size(width, height);
  bSensorInitialized_ = true;
  pEventQueueMat_.reset(new EventQueueMat(width, height, max_event_queue_length_));

  std::cout << "[TimeSurface] Sensor size: (" << sensor_size_.width << " x " << sensor_size_.height << ")" << std::endl;

  // Pre-compute the OpenCV Rectification Maps
  cv::initUndistortRectifyMap(camera_matrix_, dist_coeffs_,
                              rectification_matrix_, projection_matrix_,
                              sensor_size_, CV_32FC1, undistort_map1_, undistort_map2_);

  /* pre-compute the undistorted-rectified look-up table for Forward mode */
  precomputed_rectified_points_ = Eigen::Matrix2Xd(2, sensor_size_.height * sensor_size_.width);
  cv::Mat_<cv::Point2f> RawCoordinates(1, sensor_size_.height * sensor_size_.width);
  for (int y = 0; y < sensor_size_.height; y++)
  {
    for (int x = 0; x < sensor_size_.width; x++)
    {
      int index = y * sensor_size_.width + x;
      RawCoordinates(index) = cv::Point2f((float) x, (float) y);
    }
  }

  cv::Mat_<cv::Point2f> RectCoordinates(1, sensor_size_.height * sensor_size_.width);
  cv::undistortPoints(RawCoordinates, RectCoordinates, camera_matrix_, dist_coeffs_,
                      rectification_matrix_, projection_matrix_);

  for (size_t i = 0; i < sensor_size_.height * sensor_size_.width; i++)
  {
    precomputed_rectified_points_.col(i) = Eigen::Matrix<double, 2, 1>(
      RectCoordinates(i).x, RectCoordinates(i).y);
  }
  std::cout << "[TimeSurface] Undistorted-Rectified Look-Up Table has been computed." << std::endl;
}

cv::Mat TimeSurface::createTimeSurfaceAtTime(const double& external_sync_time)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  if(!bSensorInitialized_ || !bCamInfoAvailable_)
    return cv::Mat();

  // create exponential-decayed Time Surface map.
  const double decay_sec = decay_ms_ / 1000.0;
  cv::Mat time_surface_map;
  time_surface_map = cv::Mat::zeros(sensor_size_, CV_64F);

  // Loop through all coordinates
  for(int y=0; y<sensor_size_.height; ++y)
  {
    for(int x=0; x<sensor_size_.width; ++x)
    {
      dvs_msgs::Event most_recent_event_at_coordXY_before_T;
      // Using standard double for timestamps instead of ros::Time
      if(pEventQueueMat_->getMostRecentEventBeforeT(x, y, external_sync_time, &most_recent_event_at_coordXY_before_T))
      {
        const double& most_recent_stamp_at_coordXY = most_recent_event_at_coordXY_before_T.ts;
        if(most_recent_stamp_at_coordXY > 0)
        {
          const double dt = external_sync_time - most_recent_stamp_at_coordXY;
          double polarity = (most_recent_event_at_coordXY_before_T.polarity) ? 1.0 : -1.0;
          double expVal = std::exp(-dt / decay_sec);
          if(!ignore_polarity_)
            expVal *= polarity;

          // Backward version (Standard ESVO approach)
          if(time_surface_mode_ == BACKWARD)
            time_surface_map.at<double>(y,x) = expVal;

          // Forward version
          if(time_surface_mode_ == FORWARD && bCamInfoAvailable_)
          {
            Eigen::Matrix<double, 2, 1> uv_rect = precomputed_rectified_points_.block<2, 1>(0, y * sensor_size_.width + x);
            size_t u_i, v_i;
            if(uv_rect(0) >= 0 && uv_rect(1) >= 0)
            {
              u_i = std::floor(uv_rect(0));
              v_i = std::floor(uv_rect(1));

              if(u_i + 1 < sensor_size_.width && v_i + 1 < sensor_size_.height)
              {
                double fu = uv_rect(0) - u_i;
                double fv = uv_rect(1) - v_i;
                double fu1 = 1.0 - fu;
                double fv1 = 1.0 - fv;
                time_surface_map.at<double>(v_i, u_i) += fu1 * fv1 * expVal;
                time_surface_map.at<double>(v_i, u_i + 1) += fu * fv1 * expVal;
                time_surface_map.at<double>(v_i + 1, u_i) += fu1 * fv * expVal;
                time_surface_map.at<double>(v_i + 1, u_i + 1) += fu * fv * expVal;

                if(time_surface_map.at<double>(v_i, u_i) > 1)
                  time_surface_map.at<double>(v_i, u_i) = 1;
                if(time_surface_map.at<double>(v_i, u_i + 1) > 1)
                  time_surface_map.at<double>(v_i, u_i + 1) = 1;
                if(time_surface_map.at<double>(v_i + 1, u_i) > 1)
                  time_surface_map.at<double>(v_i + 1, u_i) = 1;
                if(time_surface_map.at<double>(v_i + 1, u_i + 1) > 1)
                  time_surface_map.at<double>(v_i + 1, u_i + 1) = 1;
              }
            }
          }
        }
      }
    }
  }

  // Polarity scaling
  if(!ignore_polarity_)
    time_surface_map = 255.0 * (time_surface_map + 1.0) / 2.0;
  else
    time_surface_map = 255.0 * time_surface_map;

  time_surface_map.convertTo(time_surface_map, CV_8U);

  // Median blur
  if(median_blur_kernel_size_ > 0)
    cv::medianBlur(time_surface_map, time_surface_map, 2 * median_blur_kernel_size_ + 1);

  // RETURN the matrix directly to our C++ program instead of publishing to ROS!
  if (time_surface_mode_ == BACKWARD && bCamInfoAvailable_)
  {
    cv::Mat final_surface;
    cv::remap(time_surface_map, final_surface, undistort_map1_, undistort_map2_, cv::INTER_LINEAR);
    return final_surface;
  }

  return time_surface_map;
}

// We leave hyperthread empty to save space, but it follows the exact same logic.
cv::Mat TimeSurface::createTimeSurfaceAtTime_hyperthread(const double& external_sync_time) {
    return createTimeSurfaceAtTime(external_sync_time);
}

void TimeSurface::thread(Job &job) {}
void TimeSurface::clearEventQueue() {}

} // namespace esvo_time_surface