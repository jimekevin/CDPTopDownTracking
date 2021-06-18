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
	SCANNERLIB_API RealsenseCameraManager();
    SCANNERLIB_API explicit RealsenseCameraManager(const std::string& bagPath);
	SCANNERLIB_API ~RealsenseCameraManager() override;
    SCANNERLIB_API bool init() override;
    SCANNERLIB_API bool init(std::string filePath) override;
	SCANNERLIB_API void stop() override;
	SCANNERLIB_API cv::Mat getColorFrame(int delayMS) override;
	SCANNERLIB_API cv::Mat getDepthFrame(int delayMS) override;
	SCANNERLIB_API cv::Mat getAveragedDepthFrame(int numFramesAveraged, std::vector<ColorSpacePoint>* colorPoints) override;
	SCANNERLIB_API double getTableAt(double x, double y) override;
	SCANNERLIB_API cv::Point3d get3DFromDepthAt(double x, double y, double depth) override;

	SCANNERLIB_API bool pollFrames();
	SCANNERLIB_API bool processFrames();
	SCANNERLIB_API rs2::video_frame getRs2ColorFrame();
	SCANNERLIB_API rs2::depth_frame getRs2DepthFrame();
	SCANNERLIB_API void applyThreshold(rs2::video_frame& other_frame, rs2::depth_frame& depth_frame, unsigned char color);

    enum SCREENSHOT_FLAGS {
        DISPLAY = 1,
        SAVE = 2,
        DISPLAY_SAVE = 3,
    };
    SCANNERLIB_API void screenshot(int step, SCREENSHOT_FLAGS flags, const std::string& screenshotPath);

    SCANNERLIB_API cv::Mat getCvColorFrame();
    // SCANNERLIB_API cv::Mat getCvDepthFrame();

    static const char *getFrameStepLabel(int step);

private:
    rs2::config cfg;
	std::shared_ptr<rs2::pipeline> pipe;
	float depth_scale;
	rs2::pipeline_profile profile;
	rs2_stream align_to;
	std::shared_ptr<rs2::align> align;
	rs2::frameset frameset;

	cv::Mat prevCvColorFrame;

	rs2::frame processedRs2ColorFrame;
	rs2::frame processedRs2DepthFrame;
	cv::Mat processedCvColorFrame;

	rs2::spatial_filter spat_filter;
	rs2::temporal_filter temp_filter;

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
        void registerCluster(Cluster cluster);
    public:
        void updateClusters(const std::vector<cv::RotatedRect> &rects);
        const Clusters& getClusters();
    };
    MultiTracker multiTracker;

  // Get random points around a 500x250 area in the center of the image
    const int m_cal_maxWidth  = 600;
    const int m_cal_maxHeight = 200;
	bool isCalibrated = false;
	cv::Mat H; // Homography matrix
	double H_zrow[4] = { 0, 0, 0, 0 };
	double callibrated_z = 10.0f;
	SCANNERLIB_API void callibrate(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame);

	static float get_depth_scale(const rs2::device& dev);
	static rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
	static bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);

	static inline double shoelace(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3);
	static inline bool pointInsideRotatedRect(const cv::Point2f& point, const cv::RotatedRect& rotatedRect);

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

	static cv::Mat convertFrameToMat(const rs2::frame& f);
	static cv::Mat convertDepthFrameToMetersMat(const rs2::depth_frame & f);

private:
	rs2::pointcloud pc;
	rs2::frameset frames;
	cv::Mat depthMat;
	cv::Mat colorMat;
};

#endif // REALSENSE_CAMERA_MANAGER_H
