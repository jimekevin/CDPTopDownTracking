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
  SCANNERLIB_API RealsenseCameraManager() = default;
  SCANNERLIB_API ~RealsenseCameraManager() override;
  SCANNERLIB_API bool Init(const std::string& bagPath = "") override;
  SCANNERLIB_API cv::Mat GetColorFrame(int delayMS) override;
  SCANNERLIB_API cv::Mat GetDepthFrame(int delayMS) override;

  SCANNERLIB_API bool PollFrames();
  SCANNERLIB_API bool ProcessFrames();
  SCANNERLIB_API rs2::video_frame GetRs2ColorFrame();
  SCANNERLIB_API rs2::depth_frame GetRs2DepthFrame();
  SCANNERLIB_API void ApplyThreshold(rs2::video_frame& other_frame, rs2::depth_frame& depth_frame, unsigned char color);

  enum SCREENSHOT_FLAGS {
    DISPLAY = 1,
    SAVE = 2,
    DISPLAY_SAVE = 3,
  };
  SCANNERLIB_API void Screenshot(int step, SCREENSHOT_FLAGS flags, const std::string& screenshotPath);

  SCANNERLIB_API cv::Mat GetCvColorFrame();
  // SCANNERLIB_API cv::Mat getCvDepthFrame();

  static const char *GetFrameStepLabel(int step);
  SCANNERLIB_API void Calibrate(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame);
  SCANNERLIB_API void Recalibrate();

private:
  typedef struct Cluster {
    cv::RotatedRect rect;
    cv::Point centroid;
    //bool assigned = false;
    int disappeared = 0;
  } Cluster;
  typedef std::map<int, Cluster> Clusters;

  class MultiTracker {
  private:
    const int maxDisappeared = 50;
    int nextClusterId = 0;
    Clusters clusters;
    void RegisterCluster(Cluster cluster);
    void DeregisterCluster(Clusters::iterator it);
  public:
    void UpdateCluster(const std::vector<cv::RotatedRect> &rects);
    const Clusters& GetClusters();
  };

  static float GetDepthScale(const rs2::device& dev);
  static rs2_stream FindStreamToAlign(const std::vector<rs2::stream_profile>& streams);
  static bool ProfileChanged(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);

  static inline double Shoelace(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3);
  static inline bool PointInsideRotatedRect(const cv::Point2f& point, const cv::RotatedRect& rotatedRect);

  MultiTracker multiTracker;

  rs2::config cfg;
  std::shared_ptr<rs2::pipeline> pipe;
  rs2::pipeline_profile profile;
  rs2_stream align_to;
  std::shared_ptr<rs2::align> align;
  rs2::frameset frameset;
  float depth_scale;

  cv::Mat prevCvColorFrame;

  rs2::frame processedRs2ColorFrame;
  rs2::frame processedRs2DepthFrame;
  cv::Mat processedCvColorFrame;

  rs2::spatial_filter spat_filter;
  rs2::temporal_filter temp_filter;

  // Get random points around a 500x250 area in the center of the image
  const int m_cal_maxWidth  = 600;
  const int m_cal_maxHeight = 200;
  bool isCalibrated = false;
  cv::Mat H; // Homography matrix
  double H_zrow[4] = { 0, 0, 0, 0 };
  double calibrated_z = 10.0f;

  // Intermediate frames
  cv::Mat frame_cvColorFrame;
  cv::Mat frame_contourColorFrame;
  cv::Mat frame_greyMat;
  cv::Mat frame_blurMat;
  cv::Mat frame_threshMat;
  cv::Mat frame_morphMat;

  // Properties
public:
  bool prop_stopped_frame = false;
  int prop_frame_step = 0;
  float prop_z_culling_back = 0.0f;
  float prop_z_culling_front = 0.25f;
  int prop_gaussian_kernel = 5;
  float prop_threshold = 0.8f;
  float prop_threshold_max = 255.0f;
  int prop_morph_kernel = 25;
  bool prop_save_screenshot = false;

  static cv::Mat ConvertFrameToMat(const rs2::frame& f);
  static cv::Mat ConvertDepthFrameToMetersMat(const rs2::depth_frame & f);

private:
  rs2::pointcloud pc;
  rs2::frameset frames;
  cv::Mat depthMat;
  cv::Mat colorMat;
};

#endif // REALSENSE_CAMERA_MANAGER_H
