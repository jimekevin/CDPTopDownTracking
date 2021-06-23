#ifndef REALSENSE_CAMERA_MANAGER_H
#define REALSENSE_CAMERA_MANAGER_H

#include <librealsense2/rs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <map>
#include <unordered_set>
#include "IMixedCameraManager.h"

class RealsenseCameraManager : public IMixedCameraManager
{
public:

  typedef struct Properties {
    bool stopped_frame = false;
    int frame_step = 0;
    float z_culling_back = 0.0f;
    float z_culling_front = 0.25f;
    int gaussian_kernel = 5;
    float threshold = 0.8f;
    float threshold_max = 255.0f;
    // int morph_kernel = 25;
    bool save_screenshot = false;
    int min_area_size = 2000;
  } Properties;
  Properties properties;

  enum SCREENSHOT_FLAGS {
    DISPLAY = 1,
    SAVE = 2,
    DISPLAY_SAVE = 3,
  };

  SCANNERLIB_API RealsenseCameraManager() = default;
  SCANNERLIB_API ~RealsenseCameraManager() override;
  SCANNERLIB_API bool Init(const std::string& bag_path = "") override;

  SCANNERLIB_API bool PollFrames();
  SCANNERLIB_API bool ProcessFrames();
  SCANNERLIB_API void ApplyThreshold(rs2::video_frame& other_frame, rs2::depth_frame& depth_frame, unsigned char color);

  SCANNERLIB_API void Calibrate(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame);
  SCANNERLIB_API void Recalibrate();

  SCANNERLIB_API void Screenshot(int step, SCREENSHOT_FLAGS flags, const std::string& screenshot_path);
  SCANNERLIB_API void ClearClusters();

  SCANNERLIB_API rs2::video_frame GetRs2ColorFrame();
  SCANNERLIB_API rs2::depth_frame GetRs2DepthFrame();
  SCANNERLIB_API cv::Mat GetCvColorFrame();
  SCANNERLIB_API cv::Mat GetCvDepthFrame();

private:

  typedef struct Cluster {
    cv::RotatedRect rect;
    cv::Point centroid;

    int disappeared = 0;
  } Cluster;
  typedef std::map<int, Cluster> Clusters;

  class MultiTracker {
  public:
    void UpdateCluster(const std::vector<cv::RotatedRect> &rects);
    const Clusters& GetClusters();
    void ClearClusters();

  private:
    void RegisterCluster(Cluster cluster);
    void DeregisterCluster(Clusters::iterator it);

    const int max_disappeared_ = 50;
    int next_cluster_id_ = 0;
    Clusters clusters_;
  };

  static const char *GetFrameStepLabel(int step);

  static float GetDepthScale(const rs2::device& dev);
  static rs2_stream FindStreamToAlign(const std::vector<rs2::stream_profile>& streams);
  static bool ProfileChanged(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);
  static cv::Mat ConvertFrameToMat(const rs2::frame& f);
  static cv::Mat ConvertDepthFrameToMetersMat(const rs2::depth_frame & f);

  static inline double Shoelace(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3);
  static inline bool PointInsideRotatedRect(const cv::Point2f& point, const cv::RotatedRect& rotated_rect);

  rs2::config cfg_;
  std::shared_ptr<rs2::pipeline> pipe_;
  rs2::pipeline_profile profile_;
  rs2_stream align_to_;
  std::shared_ptr<rs2::align> align_;
  rs2::frameset frameset_;
  float depth_scale_;

  MultiTracker multi_tracker_;

  rs2::spatial_filter spatial_filter_;
  rs2::temporal_filter temporal_filter_;
  rs2::hole_filling_filter hole_filling_filter_;

  static constexpr int calibration_max_width_  = 600;
  static constexpr int calibration_max_height_ = 200;
  bool is_calibrated_ = false;
  double H_z_row_[4] = {0, 0, 0, 0 };
  double calibrated_z_ = 10.0f;

  // Intermediate frames
  cv::Mat frame_cv_color_frame_;
  cv::Mat frame_contour_color_frame_;
  cv::Mat frame_grey_mat_;
  cv::Mat frame_blur_mat_;
  cv::Mat frame_thresh_mat_;
  cv::Mat frame_morph_mat_;
  // Final frames
  cv::Mat prev_cv_color_frame_;
  rs2::frame processed_rs_2_color_frame_;
  rs2::frame processed_rs_2_depth_frame_;
  cv::Mat processed_cv_color_frame_;
  cv::Mat processed_cv_depth_frame_;

};

#endif // REALSENSE_CAMERA_MANAGER_H
