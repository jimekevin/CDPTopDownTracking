#include "RealsenseCameraManager.h"

#include <random>
#include <iostream>

//#include <opencv2/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <utility>

//
// References
// - https://github.com/IntelRealSense/librealsense/issues/2634
//

RealsenseCameraManager::RealsenseCameraManager(const std::string& bagPath)
{
    if (bagPath.length() == 0) {
        RealsenseCameraManager();
        return;
    }

    pipe = std::make_shared<rs2::pipeline>();

    cfg.enable_device_from_file(bagPath);

    profile = pipe->start(cfg);

    depth_scale = get_depth_scale(profile.get_device());

    align_to = find_stream_to_align(profile.get_streams());
    align = std::make_shared<rs2::align>(align_to);
}

RealsenseCameraManager::RealsenseCameraManager()
{
	pipe = std::make_shared<rs2::pipeline>();

	cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 90);
	//cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
	cfg.enable_stream(RS2_STREAM_COLOR, 960, 540, RS2_FORMAT_RGB8, 60);

	profile = pipe->start(cfg);

	depth_scale = get_depth_scale(profile.get_device());

	align_to = find_stream_to_align(profile.get_streams());
	align = std::make_shared<rs2::align>(align_to);
}

void RealsenseCameraManager::MultiTracker::registerCluster(Cluster cluster) {
    clusters[nextClusterId] = std::move(cluster);
    nextClusterId++;
}

// void RealsenseCameraManager::MultiTracker::deregisterCluster(int clusterId) {
//     clusters.erase(clusterId);
// }

void RealsenseCameraManager::MultiTracker::updateClusters(const std::vector<cv::RotatedRect> &rects) {
    if (rects.empty()) {
        for (auto it = clusters.begin(), next_it = it; it != clusters.cend(); it = next_it) {
            ++next_it;
            it->second.disappeared += 1;
            if (it->second.disappeared >= maxDisappeared) {
                clusters.erase(it++);
            }
        }
        return;
    }

    // Store centroids
    std::vector<cv::Point> centroids(rects.size());
    for (auto& rect : rects) {
        centroids.emplace_back(rect.center);
    }

    // Register all new clusters
    if (clusters.empty()) {
        for (int i = 0; i < rects.size(); ++i) {
            registerCluster(Cluster{ rects[i], centroids[i] });
        }
    }
    // or match existing clusters
    else {
        // Unassign all existing clusters
        //for (auto& [clusterId, cluster] : clusters) {
        //    cluster.assigned = false;
        //}

        // Calculate the distance of each centroid to each cluster ...
        typedef struct { int centroid; int clusterId; double dist; } ccpair;
        std::vector<ccpair> dists;
        for (int i = 0; i < centroids.size(); ++i) {
            for (auto& [clusterId, cluster] : clusters) {
                dists.push_back({ i, clusterId, cv::norm(centroids[i] - cluster.centroid) });
            }
        }
        // ... and sort these pairs by distance
        std::sort(dists.begin(), dists.end(), [](ccpair c1, ccpair c2) {
          return c1.dist < c2.dist;
        });

        // Find the closest cluster for each centroid and memorize which clusters have already been visited
        std::unordered_set<int> assignedClusters;
        std::unordered_set<int> assignedCentroids;
        for (auto &&[ cindex, clusterId, dist ] : dists) {
            if (assignedCentroids.find(cindex) != assignedCentroids.cend()) {
                continue;
            }
            if (assignedClusters.find(clusterId) != assignedClusters.cend()) {
                continue;
            }
            clusters[clusterId].rect = rects[cindex];
            clusters[clusterId].centroid = centroids[cindex];
            clusters[clusterId].disappeared = 0;
            assignedCentroids.insert(cindex);
            assignedClusters.insert(clusterId);
        }


        // Assign new rects
        if (clusters.size() < rects.size()) {
            for (auto &&[ i, clusterId, dist ] : dists) {
                if (assignedCentroids.find(i) == assignedCentroids.cend()) {
                    registerCluster(Cluster{ rects[i], centroids[i] });
                }
            }
        }
        // Otherwise, remove all unassigned clusters (which have disappeared)
        else if (clusters.size() > rects.size()) {
            for (auto it = clusters.begin(), next_it = it; it != clusters.cend(); it = next_it) {
                ++next_it;
                it->second.disappeared += 1;
                if (it->second.disappeared >= maxDisappeared) {
                    clusters.erase(it++);
                }
            }
        }
    }
}

const RealsenseCameraManager::Clusters& RealsenseCameraManager::MultiTracker::getClusters() {
    return clusters;
}

bool RealsenseCameraManager::pollFrames() {
    if (!prop_stopped_frame) {
        frameset = pipe->wait_for_frames();
    }
	return true;
}

bool RealsenseCameraManager::processFrames() {

	// =========================================
	// 1. Align frames
	// =========================================

	if (profile_changed(pipe->get_active_profile().get_streams(), profile.get_streams()))
	{
		// If the profile was changed, update the align object, and also get the new device's depth scale
		profile = pipe->get_active_profile();
		align_to = find_stream_to_align(profile.get_streams());
		align = std::make_shared<rs2::align>(align_to);
		depth_scale = get_depth_scale(profile.get_device());
	}

	auto processed = align->process(frameset);

	// Trying to get both other and aligned depth frames
	rs2::video_frame other_frame = processed.first(align_to);
	rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

	// If one of them is unavailable, continue iteration
	if (!aligned_depth_frame || !other_frame) {
		return false;
	}

    auto width = other_frame.get_width();
    auto height = other_frame.get_height();

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
#pragma omp parallel for num_threads(2) default(none) shared(cvDepthFrame, aligned_filtered_depth_frame, cvColorFrame, other_frame)
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
		// prevCvDepthFrame = cvDepthFrame;
		return false;
	}
	cv::Mat contourColorFrame;
	cv::absdiff(prevCvColorFrame, cvColorFrame, contourColorFrame);

	cv::Mat greyMat; // Different mat since we reduce the channels from 3 to 1
	//cv::cvtColor(contourColorFrame, greyMat, cv::COLOR_BGR2GRAY);
    if (prop_frame_step == 1) {
        cv::cvtColor(contourColorFrame, greyMat, cv::COLOR_BGR2GRAY);
    } else {
        cv::cvtColor(cvColorFrame, greyMat, cv::COLOR_BGR2GRAY);
    }

	cv::Mat blurMat;
	cv::GaussianBlur(greyMat, blurMat, cv::Size(prop_gaussian_kernel, prop_gaussian_kernel), 0);

	cv::Mat threshMat;
	cv::threshold(blurMat, threshMat, prop_threshold * 255.0f, prop_threshold_max, cv::THRESH_BINARY);

	//cv::Mat dilateMat;
	//cv::Mat dilateKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(-1, -1));
	//cv::dilate(threshMat, dilateMat, dilateKernel, cv::Point(-1, -1), 3);

	cv::Mat morphMat;
	cv::Mat morphKernel = cv::getStructuringElement(cv::MorphShapes::MORPH_RECT, cv::Size(prop_morph_kernel, prop_morph_kernel), cv::Point(-1, -1));
	cv::morphologyEx(threshMat, morphMat, cv::MorphTypes::MORPH_OPEN, morphKernel);
	cv::morphologyEx(morphMat, morphMat, cv::MorphTypes::MORPH_CLOSE, morphKernel);

	// Cut out border so findContours does not match and merges it with the actual shapes
    cv::rectangle(morphMat, cv::Point(0, 0), cv::Point (width, height), cv::Scalar(255, 255, 255), 50);

	std::vector<std::vector<cv::Point>> contours;
	std::vector<std::vector<cv::Point>> hull;
	std::vector<double> areas;
	std::vector<double> centroids;
	std::vector<std::tuple<int, std::vector<cv::Point2f>>> boundingBoxes;
	std::vector<cv::Vec4i> hierarchy;
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
    cv::findContours(morphMat, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	hull.resize(contours.size());
    const int maxAreaSize = m_cal_maxWidth * m_cal_maxHeight;
    const int minAreaSize = 20 * 20;
    areas.clear();
	for (int i = 0; i < contours.size(); i++) {
		cv::convexHull(contours[i], hull[i]);
        auto area = cv::contourArea(hull[i]);
        areas.push_back(area);

        if (area > maxAreaSize || area < minAreaSize) {
            continue;
        }

        auto minRect = cv::minAreaRect(contours[i]);
        std::vector<cv::Point2f> box(4);
        minRect.points(box.data());
        boundingBoxes.emplace_back(std::make_tuple(i, box));
	}

    // =========================================
    // 7. Calculate approximate hand positions
    // =========================================

    std::vector<cv::RotatedRect> handBoxes;
    std::vector<cv::RotatedRect> secondaryHandBoxes;
    std::vector<std::vector<cv::Point2i>> finalContours{boundingBoxes.size()};
    std::vector<cv::RotatedRect> finalHandBoxes{boundingBoxes.size()};
    for (int i = 0; i < boundingBoxes.size(); i++) {
        auto bindex = std::get<0>(boundingBoxes[i]);
        auto &box = std::get<1>(boundingBoxes[i]);

        typedef cv::Point2f p2f;
        typedef std::vector<p2f> rect;
        typedef std::tuple<double, p2f, p2f, p2f, p2f, int> boxinfo;
        std::vector<boxinfo> sideLengths{
                {cv::norm(box[0] - box[1]), box[0], box[1], box[2], box[3], 0},
                {cv::norm(box[1] - box[2]), box[1], box[2], box[3], box[0], 1},
                {cv::norm(box[2] - box[3]), box[2], box[3], box[0], box[1], 2},
                {cv::norm(box[3] - box[0]), box[3], box[0], box[1], box[2], 3}
        };
        std::cout << std::get<5>(sideLengths[0]) << " (" << std::get<0>(sideLengths[0]) << "), ";
        std::cout << std::get<5>(sideLengths[1]) << " (" << std::get<0>(sideLengths[1]) << "), ";
        std::cout << std::get<5>(sideLengths[2]) << " (" << std::get<0>(sideLengths[2]) << "), ";
        std::cout << std::get<5>(sideLengths[3]) << " (" << std::get<0>(sideLengths[3]) << ")" << std::endl;
        std::sort(sideLengths.begin(), sideLengths.end(), [](const boxinfo &b1, const boxinfo &b2) {
          double length1 = std::get<0>(b1);
          double length2 = std::get<0>(b2);
          return length1 < length2;
        });

        std::vector<std::tuple<rect, p2f, bool>> rects;
        for (int j = 0; j < 2; j++) {
            auto[length, x1, x2, x3, x4, _] = sideLengths[j];
            auto v = x3 - x2; // direction
            auto v_hat = v / cv::norm(v);
            auto x3_new = x2 + (v_hat * length);
            auto x4_new = x1 + (v_hat * length);
            std::vector<p2f> rect{x1, x2, x3_new, x4_new};
            auto center = static_cast<cv::Point2i>(x1 + ((x3_new - x1) / 2));
            rects.emplace_back(std::make_tuple(rect, center, false));
        }
        auto imageCenterX = static_cast<float>(width) / 2.0f;
        auto imageCenterY = static_cast<float>(height) / 2.0f;
        auto imageCenter = cv::Point2f(imageCenterX, imageCenterY);

        p2f r1 = std::get<1>(rects[0]);
        p2f r2 = std::get<1>(rects[1]);
        auto dist2center1 = cv::norm(imageCenter - r1);
        auto dist2center2 = cv::norm(imageCenter - r2);
        auto dist2vertEdge1 = cv::min(r1.x, static_cast<float>(width) - r1.x);
        auto dist2vertEdge2 = cv::min(r2.x, static_cast<float>(width) - r2.x);
        auto dist2horizEdge1 = cv::min(r1.y, static_cast<float>(height) - r1.y);
        auto dist2horizEdge2 = cv::min(r2.y, static_cast<float>(height) - r2.y);

        auto score1 = dist2center1 + (imageCenter.x - dist2vertEdge1) + (imageCenter.y - dist2horizEdge1);
        auto score2 = dist2center2 + (imageCenter.x - dist2vertEdge2) + (imageCenter.y - dist2horizEdge2);

        auto rect1 = std::get<0>(rects[0]);
        cv::RotatedRect rotatedRect1(rect1[0], rect1[1], rect1[2]);
        auto rect2 = std::get<0>(rects[1]);
        cv::RotatedRect rotatedRect2(rect2[0], rect2[1], rect2[2]);
        cv::RotatedRect activeRect;
        if (score1 < score2) {
            handBoxes.emplace_back(rotatedRect1);
            secondaryHandBoxes.emplace_back(rotatedRect2);
            activeRect = rotatedRect1;
        } else {
            handBoxes.emplace_back(rotatedRect2);
            secondaryHandBoxes.emplace_back(rotatedRect1);
            activeRect = rotatedRect2;
        }

        for (const auto& c : contours[bindex]) {
            if (pointInsideRotatedRect(c, activeRect)) {
                finalContours[i].emplace_back(c);
                std::cout << "is in contour (" << bindex << ")" << std::endl;
            }
        }

        if (!finalContours[i].empty()) {
            auto minRect = cv::minAreaRect(finalContours[i]);
            finalHandBoxes.emplace_back(minRect);
        }
    }

    // =========================================
    // 8. Segmentation Localization
    // =========================================

    // Update Tracker
    //multiTracker.updateClusters(boundingBoxes);
    multiTracker.updateClusters(handBoxes);
    auto trackedClusters = multiTracker.getClusters();

    // =========================================
    // 9. Drawing
    // =========================================

    // Draw Contours
    for (int i = 0; i < contours.size(); i++) {
        if (areas[i] < maxAreaSize && areas[i] > minAreaSize) {
            //cv::drawContours(cvColorFrame, contours, i, cv::Scalar(255.0f, 0.0f, 0.0f), 2, 8, hierarchy, 0);
        }
    }

    // Draw tracked objects
    for (const auto& [clusterId, cluster] : trackedClusters) {
        std::vector<cv::Point2f> points(4);
        cluster.rect.points(points.data());
        std::vector<std::vector<cv::Point2i>> polylines{ { points[0], points[1], points[2], points[3] } };
        //cv::polylines(cvColorFrame, polylines, true, cv::Scalar(255.0f, 0.0f, 0.0f), 2, 8, 0);
        std::stringstream label;
        label << "Label: " << clusterId;
        auto color = colors[clusterId % colors.size()];
        cv::putText(cvColorFrame, label.str(), cluster.rect.boundingRect().tl(), cv::FONT_HERSHEY_SIMPLEX, 1, color);
    }

    // Draw hand boxes
    for (const auto& handBox : handBoxes) {
        std::vector<cv::Point2f> points(4);
        handBox.points(points.data());
        std::vector<std::vector<cv::Point2i>> polylines{{points[0], points[1], points[2], points[3]}};
        //cv::polylines(cvColorFrame, polylines, true, cv::Scalar(0.0f, 255.0f, 255.0f), 2, 8, 0);
    }
    for (const auto& handBox : secondaryHandBoxes) {
        std::vector<cv::Point2f> points(4);
        handBox.points(points.data());
        std::vector<std::vector<cv::Point2i>> polylines{{points[0], points[1], points[2], points[3]}};
        //cv::polylines(cvColorFrame, polylines, true, cv::Scalar(255.0f, 0.0f, 255.0f), 2, 8, 0);
    }

    // Draw final contours
    // std::cout << "finalContours: " << finalContours.size() << std::endl;
    for (const auto& finalContour : finalContours) {
        cv::polylines(cvColorFrame, finalContour, true, cv::Scalar(0.0f, 255.0f, 0.0f), 2, 8, 0);
    }

    // Draw final handboxes
    for (const auto& finalHandBox : finalHandBoxes) {
        std::vector<cv::Point2f> points(4);
        finalHandBox.points(points.data());
        std::vector<std::vector<cv::Point2i>> polylines{{points[0], points[1], points[2], points[3]}};
        cv::polylines(cvColorFrame, polylines, true, cv::Scalar(255.0f, 0.0f, 123.0f), 2, 8, 0);
    }

    // Draw calibration window
    auto rw1 = (width / 2) - (m_cal_maxWidth / 2);
    auto rw2 = (width / 2) + (m_cal_maxWidth / 2);
    auto rh1 = (height / 2) - (m_cal_maxHeight / 2);
    auto rh2 = (height / 2) + (m_cal_maxHeight / 2);
    cv::rectangle(processedCvColorFrame, cv::Rect(cv::Point(rw1, rh1), cv::Size(m_cal_maxWidth, m_cal_maxHeight)), cv::Scalar(127,0,255));

    // =========================================
	// 10. Finalize
	// =========================================

	prevCvColorFrame = cvColorFrame;
	// prevCvDepthFrame = cvDepthFrame;

	//processedCvColorFrame = cvColorFrame;
	//processedCvColorFrame = contourColorFrame;
	//processedCvColorFrame = greyMat;
	//processedCvColorFrame = blurMat;
	//processedCvColorFrame = threshMat;
	//processedCvColorFrame = dilateMat;
	//processedCvColorFrame = morphMat;
	switch (prop_frame_step) {
	    default:
	    case 0: processedCvColorFrame = cvColorFrame; break;
	    case 1: processedCvColorFrame = contourColorFrame; break;
	    case 2: processedCvColorFrame = greyMat; break;
	    case 3: processedCvColorFrame = blurMat; break;
	    case 4: processedCvColorFrame = threshMat; break;
	    case 5: processedCvColorFrame = morphMat; break;
	}
	// processedCvDepthFrame = cvDepthFrame;

	processedRs2ColorFrame = other_frame;
	processedRs2DepthFrame = aligned_filtered_depth_frame;

    // 10. Save frames in mats in case we want to display/save them later
    frame_cvColorFrame = cvColorFrame;
    frame_contourColorFrame = contourColorFrame;
    frame_greyMat = greyMat;
    frame_blurMat = blurMat;
    frame_threshMat = threshMat;
    frame_morphMat = morphMat;
	return true;
}

cv::Mat RealsenseCameraManager::getCvColorFrame() {
    return processedCvColorFrame;
}

// cv::Mat RealsenseCameraManager::getCvDepthFrame() {
//     return processedCvDepthFrame;
// }

const char* RealsenseCameraManager::getFrameStepLabel(int step) {
    switch (step) {
        default:
        case 1: return "cvColorFrame";
        case 2: return "contourColorFrame";
        case 3: return "greyMat";
        case 4: return "blurMat";
        case 5: return "threshMat";
        case 6: return "morphMat";
    }
}

inline double RealsenseCameraManager::shoelace(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3) {
    return std::fabs(0.5 * (((p2.x - p1.x) * (p3.y - p1.y)) - ((p3.x - p1.x) * (p2.y - p1.y))));
}

inline bool RealsenseCameraManager::pointInsideRotatedRect(const cv::Point2f& point, const cv::RotatedRect& rotatedRect) {
    cv::Point2f points[4];
    rotatedRect.points(points);
    const auto [rp1, rp2, rp3, rp4] = points;
    const auto area12p = shoelace(rp1, rp2, point);
    const auto area23p = shoelace(rp2, rp3, point);
    const auto area34p = shoelace(rp3, rp4, point);
    const auto area41p = shoelace(rp4, rp1, point);
    const auto areaPSum = area12p + area23p + area34p + area41p;
    const auto areaSum = shoelace(rp1, rp2, rp3) + shoelace(rp1, rp4, rp3);
    std::cout << "point = " << point << "; rp1,rp2,rp3,rp4 = " << rp1 << ", " << rp2 << ", " << rp3 << ", " << rp4 << std::endl;
    std::cout << "\tareaPSum = " << areaPSum << ", areaSum = " << areaSum << std::endl;
    return areaPSum <= areaSum;
}

void RealsenseCameraManager::callibrate(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame) {
	auto p_depth_frame = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
	auto p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

    int width = depth_frame.get_width();
    int height = depth_frame.get_height();

	float totalDepth = .0f;
	float maxDepth = std::numeric_limits<float>::min();

    // Save random pixel distances to calculate the homography plane afterwards
	std::random_device rd;
	std::mt19937 gen(rd()); 
	auto rw1 = (width / 2) - (m_cal_maxWidth / 2);
	auto rw2 = (width / 2) + (m_cal_maxWidth / 2);
	auto rh1 = (height / 2) - (m_cal_maxHeight / 2);
	auto rh2 = (height / 2) + (m_cal_maxHeight / 2);
	std::uniform_int_distribution<> distrw(rw1, rw2); 
	std::uniform_int_distribution<> distrh(rh1, rh2);
	//std::vector<xyz> pixel_depths;
	auto A_samples = cv::Mat(4, 3, CV_64F);
	auto b_samples = cv::Mat(4, 1, CV_64F);

    A_samples.at<double>(0, 0) = static_cast<double>(rw1);
    A_samples.at<double>(0, 1) = static_cast<double>(rh1);
    A_samples.at<double>(0, 2) = 1.0f;
    b_samples.at<double>(0) = depth_scale * static_cast<float>(p_depth_frame[rw1 + (rh1 * width)]);

    A_samples.at<double>(1, 0) = static_cast<double>(rw2);
    A_samples.at<double>(1, 1) = static_cast<double>(rh1);
    A_samples.at<double>(1, 2) = 1.0f;
    b_samples.at<double>(1) = depth_scale * static_cast<float>(p_depth_frame[rw2 + (rh1 * width)]);

    A_samples.at<double>(2, 0) = static_cast<double>(rw2);
    A_samples.at<double>(2, 1) = static_cast<double>(rh2);
    A_samples.at<double>(2, 2) = 1.0f;
    b_samples.at<double>(2) = depth_scale * static_cast<float>(p_depth_frame[rw2 + (rh2 * width)]);

    A_samples.at<double>(3, 0) = static_cast<double>(rw1);
    A_samples.at<double>(3, 1) = static_cast<double>(rh2);
    A_samples.at<double>(3, 2) = 1.0f;
    b_samples.at<double>(3) = depth_scale * static_cast<float>(p_depth_frame[rw1 + (rh2 * width)]);

    std::cout << "Callibration: A_samples=" << A_samples << "\n";
    std::cout << "Callibration: b_samples=" << b_samples << "\n";

    // Calculate fitted plane
	cv::Mat fitted_plane;
	auto ret = cv::solve(A_samples, b_samples, fitted_plane, cv::DECOMP_SVD);
    std::cout << "Callibration: fitted_plane=" << fitted_plane << "\n";

    // Define Z = a*X + b*Y + c
    auto project = [&](double x, double y) {
        auto a = fitted_plane.at<double>(0), b = fitted_plane.at<double>(1), c = fitted_plane.at<double>(2);
        return (a * x) + (b * y) + c;
    };

    // Get some samples (3 should be enough be the more the better I guess)
    const int maxHomographySamples = 4;
    double from_data[maxHomographySamples][3] = {
            { 100.0f, 100.0f, project(100.0f, 100.0f) },
            { -100.0f, 100.0f, project(-100.0f, 100.0f) },
            { 100.0f, -100.0f, project(100.0f, -100.0f) },
            { -100.0f, -100.0f, project(-100.0f, -100.0f) }
    };
    const auto from = cv::Mat(maxHomographySamples, 3, CV_64F, &from_data);
    double to_data[maxHomographySamples][3] = {
            { 100.0f, 100.0f, 0.0f },
            { -100.0f, 100.0f, 0.0f },
            { 100.0f, -100.0f, 0.0f },
            { -100.0f, -100.0f, 0.0f }
    };
    const auto to = cv::Mat(maxHomographySamples, 3, CV_64F, &to_data);
    std::cout << "Callibration: from=" << from << "\n";
    std::cout << "Callibration: to=" << to << "\n";
    H = cv::findHomography(from, to);
    std::cout << "Callibration: H=" << H << " (" << H.rows << "," << H.cols << ")\n";
    H_zrow[0] = H.at<double>(2, 0);
    H_zrow[1] = H.at<double>(2, 1);
    H_zrow[2] = H.at<double>(2, 2);
    H_zrow[3] = H.at<double>(2, 3);
    std::cout << "Callibration: H_zrow=[" << H_zrow[0] << "," << H_zrow[1] << "," << H_zrow[2] << "," << H_zrow[3] << "]\n";

    //double to2_data[maxHomographySamples][2] = {
            //{ 100.0f, 100.0f },
            //{ -100.0f, 100.0f },
            //{ 100.0f, -100.0f },
            //{ -100.0f, -100.0f }
            //};
    //const auto to2 = cv::Mat(maxHomographySamples, 2, CV_64F, &from_data);
    //std::cout << "Callibration[solvePnp]: from.shape=(" << from.rows << "," << from.cols << ")" << "\n";
    //std::cout << "Callibration[solvePnp]: to2.shape=(" << to2.rows << "," << to2.cols << ")" << "\n";
    //cv::Mat rvec, tvec;
    //cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type);
    //auto ret2 = cv::solvePnP(from, to2, cv::Mat::eye(3, 3, CV_64F), dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);
    //std::cout << "Callibration[solvePnp]: ret2=" << ret2 << ", rvec=" << rvec << ", tvec=" << tvec << "\n";
    //cv::Mat R(3, 3, CV_64F), RT(3, 4, CV_64F);
    //cv::Rodrigues(rvec, R);
    //std::cout << "Callibration[solvePnP]: R=" << R << "\n";
    //cv::hconcat(R, tvec, RT);
    //std::cout << "Callibration[solvePnP]: RT=" << RT << "\n";
    //
    // H_zrow[0] = R.at<double>(2, 0);
    //H_zrow[1] = R.at<double>(2, 1);
    //H_zrow[2] = R.at<double>(2, 2);
    //H_zrow[3] = R.at<double>(2, 3);

    // Get the center point of the new plane and add some small value. This is the threshold, everything
    // that is below that, i.e. under the table will be ignored
    auto centerX = static_cast<int>(width / 2);
    auto centerY = static_cast<int>(height / 2);
    auto centerZ = depth_scale * static_cast<float>(p_depth_frame[centerX + (centerY * width)]);
    std::cout << "Callibration: center=[" << centerX << "," << centerY << "," << centerZ << "]\n";
    callibrated_z = (H_zrow[0] * centerX) + (H_zrow[1] * centerY) + (H_zrow[2] * centerZ);
    callibrated_z -= 0.05;
    std::cout << "Callibration: callibrated_z=" << callibrated_z << "\n";

	isCalibrated = true;
}

void RealsenseCameraManager::applyThreshold(rs2::video_frame& other_frame, rs2::depth_frame& depth_frame, unsigned char color)
{
	//auto p_depth_frame = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
	auto p_depth_frame = reinterpret_cast<uint16_t*>(const_cast<void*>(depth_frame.get_data()));
	auto p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

	auto width = other_frame.get_width();
	auto height = other_frame.get_height();
	auto other_bpp = other_frame.get_bytes_per_pixel();

	auto zMax = callibrated_z + prop_z_culling_back;
	auto zMin = callibrated_z - prop_z_culling_front;
	zMin -= 0.25f;
//sd#pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
	for (int y = 0; y < height; y++)
	{
		auto depth_pixel_index = y * width;
		for (int x = 0; x < width; x++, ++depth_pixel_index)
		{
			// Original depth value
			auto z = depth_scale * static_cast<float>(p_depth_frame[depth_pixel_index]);
			// Altered depth value
			double pixels_distance = (H_zrow[0] * x) + (H_zrow[1] * y) + (H_zrow[2] * z) + H_zrow[3];

			// Check if the depth value is invalid (<=0) or greater than the threashold
			if (pixels_distance <= zMin || pixels_distance >= zMax)
			{
				// Calculate the offset in other frame's buffer to current pixel
				auto offset = depth_pixel_index * other_bpp;

				// Set pixel to "background" color (0x999999)
				std::memset(&p_other_frame[offset], color, other_bpp);

                // Update z index in the depth frame
                std::memset(&p_depth_frame[depth_pixel_index], static_cast<unsigned short>(pixels_distance), sizeof(uint16_t));
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

float RealsenseCameraManager::get_depth_scale(const rs2::device& dev)
{
	// Go over the device's sensors
	for (rs2::sensor& sensor : dev.query_sensors())
	{
		// Check if the sensor if a depth sensor
		if (auto dpt = sensor.as<rs2::depth_sensor>())
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
	for (const auto& sp : streams)
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
    pipe->stop();
}

bool RealsenseCameraManager::init() {
	pipe = std::make_shared<rs2::pipeline>();
	cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 90);
	//cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
	cfg.enable_stream(RS2_STREAM_COLOR, 960, 540, RS2_FORMAT_RGB8, 60);
	return (bool)pipe->start(cfg);
}

bool RealsenseCameraManager::init(std::string filePath) {
	pipe = std::make_shared<rs2::pipeline>();
	cfg.enable_device_from_file(filePath);
	//cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 90);
	//cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
	//cfg.enable_stream(RS2_STREAM_COLOR, 960, 540, RS2_FORMAT_RGB8, 60);
	return (bool)pipe->start(cfg);
}

void RealsenseCameraManager::stop() {
}

cv::Mat RealsenseCameraManager::getColorFrame(int delayMS) {
	return colorMat;
}

cv::Mat RealsenseCameraManager::getDepthFrame(int delayMS) {
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

void RealsenseCameraManager::screenshot(int step, SCREENSHOT_FLAGS flags, const std::string& screenshotPath) {
    cv::Mat mat;
    switch (step) {
        default:
        case 0: mat = frame_cvColorFrame; break;
        case 1: mat = frame_contourColorFrame; break;
        case 2: mat = frame_greyMat; break;
        case 3: mat = frame_blurMat; break;
        case 4: mat = frame_threshMat; break;
        case 5: mat = frame_morphMat; break;
    }

    if (flags == SAVE || flags == DISPLAY_SAVE) {
        std::stringstream imagePath;
        auto time = std::time(nullptr);
        imagePath << screenshotPath << time << ".png";
        std::cout << "Saved screenshot of '" << getFrameStepLabel(step) << "' to '" << imagePath.str() << "'";
        cv::imwrite(imagePath.str(), mat);
    }

    if (flags == DISPLAY || flags == DISPLAY_SAVE) {
        std::stringstream windowName;
        windowName << "Display '" << getFrameStepLabel(step) << "'";
        cv::imshow(windowName.str(), mat);
        cv::waitKey(0);
        cv::destroyWindow(windowName.str());
    }
}