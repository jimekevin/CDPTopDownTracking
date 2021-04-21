#include "RealsenseCameraManager.h"

#include <random>

//
// References
// - https://github.com/IntelRealSense/librealsense/issues/2634
//

RealsenseCameraManager::RealsenseCameraManager()
{
	pipe = std::make_shared<rs2::pipeline>();
	rs2::config cfg;

	//cfg.enable_device_from_file("C:\\Projects\\CDPTopDownTracking\\TopDownTracking\\data\\20210401_191338.bag");
	cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 90);
	//cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
	cfg.enable_stream(RS2_STREAM_COLOR, 960, 540, RS2_FORMAT_RGB8, 60);

	profile = pipe->start(cfg);

	depth_scale = get_depth_scale(profile.get_device());

	align_to = find_stream_to_align(profile.get_streams());
	//rs2::align align(align_to);
	align = std::make_shared<rs2::align>(align_to);

	// Define a variable for controlling the distance to clip
	//float depth_clipping_distance = 1.f;
}

bool RealsenseCameraManager::pollFrames() {
	frameset = pipe->wait_for_frames();
	return true;
}

bool RealsenseCameraManager::processFrames() {

	// =========================================
	// 1. Align frames
	// =========================================

	if (profile_changed(pipe->get_active_profile().get_streams(), profile.get_streams()))
	{
		//If the profile was changed, update the align object, and also get the new device's depth scale
		profile = pipe->get_active_profile();
		align_to = find_stream_to_align(profile.get_streams());
		align = std::make_shared<rs2::align>(align_to);
		depth_scale = get_depth_scale(profile.get_device());
	}

	auto processed = align->process(frameset);

	// Trying to get both other and aligned depth frames
	rs2::video_frame other_frame = processed.first(align_to);
	rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

	//If one of them is unavailable, continue iteration
	if (!aligned_depth_frame || !other_frame)
	{
		return false;
	}

	// =========================================
	// 2. Filters (spatial then temporal) https://dev.intelrealsense.com/docs/depth-post-processing
	// =========================================

	rs2::depth_frame aligned_filtered_depth_frame = aligned_depth_frame;
	aligned_filtered_depth_frame = spat_filter.process(aligned_filtered_depth_frame);
	aligned_filtered_depth_frame = temp_filter.process(aligned_filtered_depth_frame);

	// =========================================
	// 3. Optional: Calibrate z threshold if needed (maybe x and y as well?)
	// =========================================

	if (!isCalibrated) {
		callibrate(other_frame, aligned_filtered_depth_frame);
		// TODO: ignore calibration for now
		clippingDistances.z.max = 1.5f;
	}

	// =========================================
	// 4. Threshold
	// =========================================

	const int colorWhite = 0xff;
	applyThreshold(other_frame, aligned_filtered_depth_frame, colorWhite);

	// =========================================
	// 5. Convert to OpenCV
	// =========================================

	cv::Mat cvDepthFrame, cvColorFrame;
#pragma omp parallel for num_threads(2)
	for (int i = 0; i < 2; ++i) {
		if (i == 0) {
			cvDepthFrame = convertDepthFrameToMetersMat(aligned_filtered_depth_frame);
		}
		else {
			cvColorFrame = convertFrameToMat(other_frame);
		}
	}

	// =========================================
	// 6. Contour Detection
	// =========================================

	if (prevCvColorFrame.cols == 0) {
		prevCvColorFrame = cvColorFrame;
		prevCvDepthFrame = cvDepthFrame;
		return false;
	}
	cv::Mat contourColorFrame;
	cv::absdiff(prevCvColorFrame, cvColorFrame, contourColorFrame);

	cv::Mat greyMat; // Differennt mat since we reduce the channels from 3 to 1
	//cv::cvtColor(contourColorFrame, greyMat, cv::COLOR_BGR2GRAY);
	cv::cvtColor(cvColorFrame, greyMat, cv::COLOR_BGR2GRAY);

	cv::Mat blurMat;
	cv::GaussianBlur(greyMat, blurMat, cv::Size(5, 5), 0);

	cv::Mat threshMat;
	cv::threshold(blurMat, threshMat, 0.2f * 255, 255.0f, cv::THRESH_BINARY);

	//cv::Mat dilateMat;
	//cv::Mat dilateKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(-1, -1));
	//cv::dilate(threshMat, dilateMat, dilateKernel, cv::Point(-1, -1), 3);

	cv::Mat morphMat;
	cv::Mat morphKernel = cv::getStructuringElement(cv::MorphShapes::MORPH_RECT, cv::Size(5, 5), cv::Point(-1, -1));
	cv::morphologyEx(threshMat, morphMat, cv::MorphTypes::MORPH_OPEN, morphKernel);
	cv::morphologyEx(morphMat, morphMat, cv::MorphTypes::MORPH_CLOSE, morphKernel);

	std::vector<std::vector<cv::Point>> contours;
	std::vector<std::vector<cv::Point>> hull;
	std::vector<double> areas;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(morphMat, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	auto redColor = cv::Scalar(1.0f, 0.0f, 0.0f, 1.0f);
	int idx = 0;
	std::vector<cv::Scalar> colors = {
		cv::Scalar(1.0f, 0.0f, 0.0f) * 255, //, 1.0f),
		cv::Scalar(0.0f, 1.0f, 0.0f) * 255, //, 1.0f),
		cv::Scalar(0.0f, 0.0f, 1.0f) * 255, //, 1.0f),
		cv::Scalar(1.0f, 1.0f, 0.0f) * 255, //, 1.0f),
		cv::Scalar(1.0f, 0.0f, 1.0f) * 255, //, 1.0f),
		cv::Scalar(0.0f, 1.0f, 1.0f) * 255, //, 1.0f),
		cv::Scalar(0.5f, 1.0f, 0.0f) * 255, //, 1.0f),
		cv::Scalar(0.5f, 0.0f, 1.0f) * 255, //, 1.0f),
		cv::Scalar(1.0f, 0.5f, 0.0f) * 255, //, 1.0f),
		cv::Scalar(1.0f, 0.0f, 0.5f) * 255, //, 1.0f),
	};
	hull.resize(contours.size());
	for (int i = 0; i < contours.size(); i++) {
		cv::convexHull(contours[i], hull[i]);
	}
	areas.clear();
	for (int i = 0; i < contours.size(); i++) {
		auto area = cv::contourArea(hull[i]);
		areas.push_back(area);
		auto contsize = contours[i].size();
		if (contours[i].size() > 50) {
			continue;
		}
		if (area > 1000) {
			continue;
		}
		auto color = colors[i % colors.size()];
		cv::drawContours(cvColorFrame, hull, i, color, cv::FILLED, 8);
		cv::drawContours(cvColorFrame, contours, i, color, 2, 8, hierarchy, 0);
		auto bound = cv::boundingRect(contours[i]);
		std::stringstream label;
		label << "Label: " << i;
		cv::putText(cvColorFrame, label.str(), cv::Point(bound.x, bound.y), cv::FONT_HERSHEY_SIMPLEX, 1, color);
	}

	// =========================================
	// 7. Write return values
	// =========================================

	prevCvColorFrame = cvColorFrame;
	prevCvDepthFrame = cvDepthFrame;

	//processedCvColorFrame = cvColorFrame;
	//processedCvColorFrame = contourColorFrame;
	//processedCvColorFrame = greyMat;
	//processedCvColorFrame = blurMat;
	//processedCvColorFrame = threshMat;
	//processedCvColorFrame = dilateMat;
	//processedCvColorFrame = morphMat;
	processedCvColorFrame = cvColorFrame;
	processedCvDepthFrame = cvDepthFrame;

	processedRs2ColorFrame = other_frame;
	processedRs2DepthFrame = aligned_filtered_depth_frame;
}

void RealsenseCameraManager::callibrate(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame) {
	const uint16_t* p_depth_frame = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
	uint8_t* p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

	int width = other_frame.get_width();
	int height = other_frame.get_height();

	// Get random points around a 200x200 area in the center of the image
	const int maxWidth  = 200;
	const int maxHeight = 200;
	const int maxPoints = 50;
	float totalDepth = .0f;

	std::random_device rd;
	std::mt19937 gen(rd()); 
	auto rw1 = (width / 2) - (maxWidth / 2);
	auto rw2 = (width / 2) + (maxWidth / 2);
	auto rh1 = (height / 2) - (maxHeight / 2);
	auto rh2 = (height / 2) + (maxHeight / 2);
	std::uniform_int_distribution<> distrw(rw1, rw2); 
	std::uniform_int_distribution<> distrh(rh1, rh2); 

#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < maxPoints; ++i) {
		auto rw = distrw(gen);
		auto rh = distrh(gen);
		auto randomIndex = rw + (rh * width);
		// Get the depth value of the current pixel
		auto pixels_distance = depth_scale * p_depth_frame[randomIndex];
#pragma omp atomic
		totalDepth += pixels_distance;
	}
	auto meanDepth = totalDepth / maxPoints;

	clippingDistances.z.max = meanDepth;
	clippingDistances.z.min = 0.0f;

	isCalibrated = true;
}

void RealsenseCameraManager::applyThreshold(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame, unsigned char color)
{
	auto p_depth_frame = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
	auto p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

	auto width = other_frame.get_width();
	auto height = other_frame.get_height();
	auto other_bpp = other_frame.get_bytes_per_pixel();

#pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
	for (int y = 0; y < height; y++)
	{
		auto depth_pixel_index = y * width;
		for (int x = 0; x < width; x++, ++depth_pixel_index)
		{
			// Get the depth value of the current pixel
			auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];

			// Check if the depth value is invalid (<=0) or greater than the threashold
			if (pixels_distance <= clippingDistances.z.min || pixels_distance > clippingDistances.z.max)
			{
				// Calculate the offset in other frame's buffer to current pixel
				auto offset = depth_pixel_index * other_bpp;

				// Set pixel to "background" color (0x999999)
				std::memset(&p_other_frame[offset], color, other_bpp);
			}
		}
	}
}

rs2::video_frame RealsenseCameraManager::getRs2ColorFrame() {
	return processedRs2ColorFrame.as<rs2::video_frame>();
}

rs2::depth_frame RealsenseCameraManager::getRs2DepthFrame() {
	return processedRs2DepthFrame.as<rs2::depth_frame>();
}

cv::Mat RealsenseCameraManager::getCvColorFrame() {
	return processedCvColorFrame;
}

cv::Mat RealsenseCameraManager::getCvDepthFrame() {
	return processedCvDepthFrame;
}

float RealsenseCameraManager::get_depth_scale(rs2::device dev)
{
	// Go over the device's sensors
	for (rs2::sensor& sensor : dev.query_sensors())
	{
		// Check if the sensor if a depth sensor
		if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
		{
			return dpt.get_depth_scale();
		}
	}
	throw std::runtime_error("Device does not have a depth sensor");
}

rs2_stream RealsenseCameraManager::find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
	//Given a vector of streams, we try to find a depth stream and another stream to align depth with.
	//We prioritize color streams to make the view look better.
	//If color is not available, we take another stream that (other than depth)
	rs2_stream align_to = RS2_STREAM_ANY;
	bool depth_stream_found = false;
	bool color_stream_found = false;
	for (rs2::stream_profile sp : streams)
	{
		rs2_stream profile_stream = sp.stream_type();
		if (profile_stream != RS2_STREAM_DEPTH)
		{
			if (!color_stream_found)         //Prefer color
				align_to = profile_stream;

			if (profile_stream == RS2_STREAM_COLOR)
			{
				color_stream_found = true;
			}
		}
		else
		{
			depth_stream_found = true;
		}
	}

	if (!depth_stream_found)
		throw std::runtime_error("No Depth stream available");

	if (align_to == RS2_STREAM_ANY)
		throw std::runtime_error("No stream found to align with Depth");

	return align_to;
}

bool RealsenseCameraManager::profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
	for (auto&& sp : prev)
	{
		//If previous profile is in current (maybe just added another)
		auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
		if (itr == std::end(current)) //If it previous stream wasn't found in current
		{
			return true;
		}
	}
	return false;
}

RealsenseCameraManager::~RealsenseCameraManager()
{
	stop();
}


bool RealsenseCameraManager::init() {
	pipe = std::make_shared<rs2::pipeline>();
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 90);
	//cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
	cfg.enable_stream(RS2_STREAM_COLOR, 960, 540, RS2_FORMAT_RGB8, 60);
	return (bool)pipe->start(cfg);
}

bool RealsenseCameraManager::init(std::string filePath) {
	pipe = std::make_shared<rs2::pipeline>();
	rs2::config cfg;
	cfg.enable_device_from_file(filePath);
	//cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 90);
	//cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
	//cfg.enable_stream(RS2_STREAM_COLOR, 960, 540, RS2_FORMAT_RGB8, 60);
	return (bool)pipe->start(cfg);
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

/*
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
		}
		else {
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
*/

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
		//return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
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