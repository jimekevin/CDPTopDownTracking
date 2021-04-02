#include "RealsenseCameraManager.h"

RealsenseCameraManager::RealsenseCameraManager()
{
}

RealsenseCameraManager::~RealsenseCameraManager()
{
	stop();
}
bool RealsenseCameraManager::init() {
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 90);
    //cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 960, 540, RS2_FORMAT_RGB8, 60);
    return (bool) pipe->start(cfg);
}

bool RealsenseCameraManager::init(std::string filePath) {
    pipe = std::make_shared<rs2::pipeline>();
	rs2::config cfg;
    cfg.enable_device_from_file(filePath);
	//cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 90);
	//cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
	//cfg.enable_stream(RS2_STREAM_COLOR, 960, 540, RS2_FORMAT_RGB8, 60);
	return (bool) pipe->start(cfg);
}

void RealsenseCameraManager::stop() {
	pipe->stop();
}

cv::Mat RealsenseCameraManager::getColorFrame([[maybe_unused]] int delayMS) {
	return colorMat;
}

cv::Mat RealsenseCameraManager::getDepthFrame([[maybe_unused]] int delayMS) {
	return depthMat;
}

cv::Mat RealsenseCameraManager::getAveragedDepthFrame(int numFramesAveraged, std::vector<ColorSpacePoint>* colorPoints) {
	return cv::Mat::zeros(cv::Size(1, 1), CV_32F);
}

double RealsenseCameraManager::getTableAt(double x, double y) {
	return 0;
}
cv::Point3d RealsenseCameraManager::get3DFromDepthAt(double x, double y, double depth) {
	return cv::Point3d(0.0, 0.0, 0.0);
}

bool RealsenseCameraManager::pollFrames() {
	return pipe->poll_for_frames(&frames);

	//if (!pipe->poll_for_frames(&frames)) {
	//	return false;
	//}

	//// Get frames
	//auto colorFrame = frames.get_color_frame();
	//auto depthFrame = frames.get_depth_frame();
	//
	//// Convert depth frame from 16UC1 to cv::Mat with float 64F
	//auto depthMat = cv::Mat(cv::Size(depthFrame.get_width(), depthFrame.get_height()), CV_16UC1, (void*)depthFrame.get_data(), cv::Mat::AUTO_STEP);
	//depthMat.convertTo(depthMat, CV_64F);
	//depthMat = depthMat * depthFrame.get_units();
	//// Convert color frame to cv::Mat
	//colorMat = cv::Mat(cv::Size(colorFrame.get_width(), colorFrame.get_height()), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);

	//return true;
}

RealsenseCameraManager::RenderSet RealsenseCameraManager::processFrames() {
	//auto align_to = rs2_stream::RS2_STREAM_COLOR;
	//auto align_to = rs2_stream::RS2_STREAM_DEPTH;
	//rs2::align align(align_to);
	//auto alignedFrames = align.process(frames);

	auto colorFrame = frames.get_color_frame();
	auto depthFrame = frames.get_depth_frame();
	pc.map_to(colorFrame);
	auto points = pc.calculate(depthFrame);

	rs2::frame colorFiltered = colorFrame;
	rs2::frame depthFiltered = depthFrame;
	for (auto &&filter : filters) {
		if (filter.enabled) {
			if (filter.type == rs2_stream::RS2_STREAM_COLOR) {
				colorFiltered = filter.filter.process(colorFiltered);
			}
			else {
				depthFiltered = filter.filter.process(depthFiltered);
			}
		}
	}

	//rs2::colorizer c;
	//colorFrame = c.process(depthFrame);

	cv::Mat cvDepthFrame, cvColorFrame;
//#pragma omp parallel for num_threads(2)
    for (int i = 0; i < 2; ++i) {
	    if (i == 0) {
            cvDepthFrame = convertDepthFrameToMetersMat(depthFrame);
	    } else {
	        cvColorFrame = convertFrameToMat(colorFrame);
	    }
	}

    for (auto &&task : tasks) {
        if (task.enabled) {
            task.task->process(cvDepthFrame, cvColorFrame);
        }
    }

	return RenderSet{ points, depthFrame, colorFrame, cvDepthFrame, cvColorFrame };
}

int RealsenseCameraManager::addFilter(Filter filter) {
	filters.emplace_back(filter);
	return filters.size() - 1;
}

void RealsenseCameraManager::enableFilter(int filterId, bool enabled) {
	if (filterId >= 0 && filterId < filters.size()) {
		filters[filterId].enabled = enabled;
	}
}

int RealsenseCameraManager::addTask(Task task) {
    tasks.emplace_back(task);
    return tasks.size() - 1;
}

void RealsenseCameraManager::enableTask(int taskId, bool enabled) {
    if (taskId >= 0 && taskId < tasks.size()) {
        tasks[taskId].enabled = enabled;
    }
}

// TODO: Remove
cv::Mat RealsenseCameraManager::convertFrameToMat(const rs2::frame& f)
{
	using namespace cv;
	using namespace rs2;

	auto vf = f.as<video_frame>();
	const int w = vf.get_width();
	const int h = vf.get_height();

	if (f.get_profile().format() == RS2_FORMAT_BGR8)
	{
		return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
	}
	else if (f.get_profile().format() == RS2_FORMAT_RGB8)
	{
		auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
		Mat r_bgr;
		cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
		return r_bgr;
	}
	else if (f.get_profile().format() == RS2_FORMAT_Z16)
	{
		return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
	}
	else if (f.get_profile().format() == RS2_FORMAT_Y8)
	{
		return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
	}
	else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
	{
		return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
	}

	throw std::runtime_error("Frame format is not supported yet!");
}

// Converts depth frame to a matrix of doubles with distances in meters
cv::Mat RealsenseCameraManager::convertDepthFrameToMetersMat(const rs2::depth_frame & f)
{
	cv::Mat dm = convertFrameToMat(f);
	dm.convertTo(dm, CV_64F);
	dm = dm * f.get_units();
	return dm;
}