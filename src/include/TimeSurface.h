#ifndef esvo_time_surface_H_
#define esvo_time_surface_H_

// --- REMOVED ALL ROS 1 INCLUDES ---
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Eigen>

#include <deque>
#include <mutex>
#include <memory>
#include <vector>

// --- THE FAKE DVS MESSAGE BRIDGE ---
// This prevents us from having to rewrite ESVO's entire internal math logic
namespace dvs_msgs
{
  struct Event {
    uint16_t x;
    uint16_t y;
    double ts; // Changed from ros::Time to standard double (seconds)
    bool polarity;
  };
}

namespace esvo_time_surface
{
#define NUM_THREAD_TS 1
using EventQueue = std::deque<dvs_msgs::Event>;

// --- NEW STRUCT TO REPLACE ros::NodeHandle ---
struct TimeSurfaceParams {
    int width;
    int height;
    double decay_ms;
    bool ignore_polarity;
    int median_blur_kernel_size;
    int max_event_queue_length;
    int events_maintained_size;

    // We pass the calibration directly now instead of waiting for a ROS topic
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Mat rectification_matrix;
    cv::Mat projection_matrix;
};

class EventQueueMat
{
public:
  EventQueueMat(int width, int height, int queueLen)
  {
    width_ = width;
    height_ = height;
    queueLen_ = queueLen;
    eqMat_ = std::vector<EventQueue>(width_ * height_, EventQueue());
  }

  void insertEvent(const dvs_msgs::Event& e)
  {
    if(!insideImage(e.x, e.y))
      return;
    else
    {
      EventQueue& eq = getEventQueue(e.x, e.y);
      eq.push_back(e);
      while(eq.size() > queueLen_)
        eq.pop_front();
    }
  }

  // Changed ros::Time to double
  bool getMostRecentEventBeforeT(
    const size_t x,
    const size_t y,
    const double& t,
    dvs_msgs::Event* ev)
  {
    if(!insideImage(x, y))
      return false;

    EventQueue& eq = getEventQueue(x, y);
    if(eq.empty())
      return false;

    for(auto it = eq.rbegin(); it != eq.rend(); ++it)
    {
      const dvs_msgs::Event& e = *it;
      if(e.ts < t)
      {
        *ev = *it;
        return true;
      }
    }
    return false;
  }

  void clear()
  {
    eqMat_.clear();
  }

  bool insideImage(const size_t x, const size_t y)
  {
    return !(x < 0 || x >= width_ || y < 0 || y >= height_);
  }

  inline EventQueue& getEventQueue(const size_t x, const size_t y)
  {
    return eqMat_[x + width_ * y];
  }

  size_t width_;
  size_t height_;
  size_t queueLen_;
  std::vector<EventQueue> eqMat_;
};

class TimeSurface
{
  struct Job
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EventQueueMat* pEventQueueMat_;
    cv::Mat* pTimeSurface_;
    size_t start_col_, end_col_;
    size_t start_row_, end_row_;
    size_t i_thread_;
    double external_sync_time_; // Changed from ros::Time
    double decay_sec_;
  };

public:
  // Replaced ros::NodeHandle with our clean parameter struct
  TimeSurface(const TimeSurfaceParams& params);
  virtual ~TimeSurface();

  // --- MOVED TO PUBLIC SO OUR ROS 2 BRIDGE CAN ACCESS THEM ---
  std::shared_ptr<EventQueueMat> pEventQueueMat_;

  // Expose the generation function so our ROS 2 bridge can trigger it manually
  cv::Mat createTimeSurfaceAtTime(const double& external_sync_time);

private:
  // core
  void init(int width, int height);
  cv::Mat createTimeSurfaceAtTime_hyperthread(const double& external_sync_time); // Changed ros::Time
  void thread(Job& job);

  // utils
  void clearEventQueue();

  // calibration parameters
  cv::Mat camera_matrix_, dist_coeffs_;
  cv::Mat rectification_matrix_, projection_matrix_;
  std::string distortion_model_;
  cv::Mat undistort_map1_, undistort_map2_;
  Eigen::Matrix2Xd precomputed_rectified_points_;

  // online parameters
  bool bCamInfoAvailable_;
  cv::Size sensor_size_;
  double sync_time_; // Changed from ros::Time
  bool bSensorInitialized_;

  // offline parameters
  double decay_ms_;
  bool ignore_polarity_;
  int median_blur_kernel_size_;
  int max_event_queue_length_;
  int events_maintained_size_;

  // containers
  EventQueue events_;

  // thread mutex
  std::mutex data_mutex_;

  enum TimeSurfaceMode
  {
    BACKWARD,
    FORWARD
  } time_surface_mode_;
};

} // namespace esvo_time_surface
#endif // esvo_time_surface_H_