#include "RealsenseCameraManager.h"

#include <random>
#include <iostream>
#include <fstream>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

void RealsenseCameraManager::MultiTracker::RegisterCluster(Cluster cluster) {
  clusters_[next_cluster_id_] = std::move(cluster);
  next_cluster_id_++;
}

void RealsenseCameraManager::MultiTracker::DeregisterCluster(Clusters::iterator it) {
  clusters_.erase(it);
}

void RealsenseCameraManager::MultiTracker::UpdateCluster(const std::vector<cv::RotatedRect> &rects) {
  // Remove all clusters that disappeared
  if (rects.empty()) {
    for (auto it = clusters_.begin(), next_it = it; it != clusters_.cend(); it = next_it) {
      ++next_it;
      it->second.disappeared += 1;
      if (it->second.disappeared >= max_disappeared_) {
        DeregisterCluster(it++);
      }
    }
    return;
  }

  // Store centroids
  std::vector<cv::Point> centroids;
  for (auto& rect : rects) {
    centroids.emplace_back(rect.center);
  }

  // Register all new clusters
  if (clusters_.empty()) {
    for (int i = 0; i < rects.size(); ++i) {
      RegisterCluster(Cluster{rects[i], centroids[i]});
    }
  }
  // or match existing clusters
  else {
    // Calculate the distance of each centroid to each cluster ...
    typedef struct { int centroid; int cluster_id_; double dist; } ccpair;
    std::vector<ccpair> dists;
    for (int i = 0; i < centroids.size(); ++i) {
      for (auto& [clusterId, cluster] : clusters_) {
        dists.push_back({ i, clusterId, cv::norm(centroids[i] - cluster.centroid) });
      }
    }
    // ... and sort these pairs by distance
    std::sort(dists.begin(), dists.end(), [](ccpair c1, ccpair c2) {
      return c1.dist < c2.dist;
    });

    // Find the closest cluster for each centroid and memorize which clusters have already been visited
    std::unordered_set<int> assigned_clusters;
    std::unordered_set<int> assigned_centroids;
    for (auto &&[ cindex, clusterId, dist ] : dists) {
      if (assigned_centroids.find(cindex) != assigned_centroids.cend()) {
        continue;
      }
      if (assigned_clusters.find(clusterId) != assigned_clusters.cend()) {
        continue;
      }
      clusters_[clusterId].rect = rects[cindex];
      clusters_[clusterId].centroid = centroids[cindex];
      clusters_[clusterId].disappeared = 0;
      assigned_centroids.insert(cindex);
      assigned_clusters.insert(clusterId);
    }

    // Assign new rects
    if (clusters_.size() < rects.size()) {
      for (auto &&[ i, clusterId, dist ] : dists) {
        if (assigned_centroids.find(i) == assigned_centroids.cend()) {
          RegisterCluster(Cluster{rects[i], centroids[i]});
        }
      }
    }
    // Otherwise, remove all unassigned clusters (which have disappeared)
    else if (clusters_.size() > rects.size()) {
      for (auto it = clusters_.begin(), next_it = it; it != clusters_.cend(); it = next_it) {
        ++next_it;
        it->second.disappeared += 1;
        if (it->second.disappeared >= max_disappeared_) {
          DeregisterCluster(it++);
        }
      }
    }
  }
}

bool RealsenseCameraManager::Init(const std::string& bag_path)
{
  pipe_ = std::make_shared<rs2::pipeline>();

  std::ifstream infile(bag_path.c_str());
  if (bag_path.length() == 0 || bag_path == "LIVE" || !infile.good()) {
    cfg_.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 90);
    // cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
    cfg_.enable_stream(RS2_STREAM_COLOR, 960, 540, RS2_FORMAT_RGB8, 60);
  } else {
    cfg_.enable_device_from_file(bag_path);
  }

  profile_ = pipe_->start(cfg_);

  depth_scale_ = GetDepthScale(profile_.get_device());

  align_to_ = FindStreamToAlign(profile_.get_streams());
  align_ = std::make_shared<rs2::align>(align_to_);

  return true;
}

const RealsenseCameraManager::Clusters& RealsenseCameraManager::MultiTracker::GetClusters() {
  return clusters_;
}

bool RealsenseCameraManager::PollFrames() {
  if (!properties.stopped_frame) {
    frameset_ = pipe_->wait_for_frames();
  }
  return true;
}

bool RealsenseCameraManager::ProcessFrames() {

  // =========================================
  // 1. Align frames
  // =========================================

  if (ProfileChanged(pipe_->get_active_profile().get_streams(), profile_.get_streams()))
  {
    // If the profile was changed, update the align object, and also get the new device's depth scale
    profile_ = pipe_->get_active_profile();
    align_to_ = FindStreamToAlign(profile_.get_streams());
    align_ = std::make_shared<rs2::align>(align_to_);
    depth_scale_ = GetDepthScale(profile_.get_device());
  }

  auto processed = align_->process(frameset_);

  // Trying to get both other and aligned depth frames
  rs2::video_frame other_frame = processed.first(align_to_);
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
  aligned_filtered_depth_frame = spatial_filter_.process(aligned_filtered_depth_frame);
  aligned_filtered_depth_frame = temporal_filter_.process(aligned_filtered_depth_frame);

  // =========================================
  // 3. Optional: Calibrate z threshold if needed (maybe x and y as well?)
  // =========================================

  if (!is_calibrated_) {
    Calibrate(other_frame, aligned_filtered_depth_frame);
  }

  // =========================================
  // 4. Threshold
  // =========================================

  const int colorWhite = 0xff;
  ApplyThreshold(other_frame, aligned_filtered_depth_frame, colorWhite);

  // =========================================
  // 5. Convert to OpenCV
  // =========================================

  cv::Mat cvDepthFrame, cv_color_frame;
#pragma omp parallel for num_threads(2) default(none) shared(cvDepthFrame, aligned_filtered_depth_frame, cv_color_frame, other_frame)
  for (int i = 0; i < 2; ++i) {
    if (i == 0) {
      cvDepthFrame = ConvertDepthFrameToMetersMat(aligned_filtered_depth_frame);
    }
    else {
      cv_color_frame = ConvertFrameToMat(other_frame);
    }
  }

  // =========================================
  // 6. Contour Detection
  // =========================================

  if (prev_cv_color_frame_.cols == 0) {
    prev_cv_color_frame_ = cv_color_frame;
    // prevCvDepthFrame = cvDepthFrame;
    return false;
  }
  cv::Mat contour_color_frame;
  cv::absdiff(prev_cv_color_frame_, cv_color_frame, contour_color_frame);

  cv::Mat greyMat; // Different mat since we reduce the channels from 3 to 1
  //cv::cvtColor(contourColorFrame, greyMat, cv::COLOR_BGR2GRAY);
  if (properties.frame_step == 1) {
    cv::cvtColor(contour_color_frame, greyMat, cv::COLOR_BGR2GRAY);
  } else {
    cv::cvtColor(cv_color_frame, greyMat, cv::COLOR_BGR2GRAY);
  }

  cv::Mat blurMat;
  cv::GaussianBlur(greyMat, blurMat, cv::Size(properties.gaussian_kernel, properties.gaussian_kernel), 0);

  cv::Mat threshMat;
  cv::threshold(blurMat, threshMat, properties.threshold * 255.0f, properties.threshold_max, cv::THRESH_BINARY);

  // Dilation brings no performance increase
  // cv::Mat dilateMat;
  // cv::Mat dilateKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(-1, -1));
  // cv::dilate(threshMat, dilateMat, dilateKernel, cv::Point(-1, -1), 3);

  // Morph Mat is really slow on the main cdp station
  cv::Mat morphMat = threshMat;
  // cv::Mat morphMat;
  // cv::Mat morphKernel = cv::getStructuringElement(cv::MorphShapes::MORPH_RECT, cv::Size(properties.morph_kernel, properties.morph_kernel), cv::Point(-1, -1));
  // cv::morphologyEx(threshMat, morphMat, cv::MorphTypes::MORPH_OPEN, morphKernel);
  // cv::morphologyEx(morphMat, morphMat, cv::MorphTypes::MORPH_CLOSE, morphKernel);

  // Cut out border so findContours does not match and merges it with the actual shapes
  cv::rectangle(morphMat, cv::Point(0, 0), cv::Point (width, height), cv::Scalar(255, 255, 255), 50);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<std::vector<cv::Point>> hull;
  std::vector<double> areas;
  std::vector<double> centroids;
  std::vector<std::tuple<int, std::vector<cv::Point2f>>> bounding_boxes;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<cv::Scalar> colors = {
      cv::Scalar(255.0f, 0.0f, 0.0f),
      cv::Scalar(0.0f, 255.0f, 0.0f),
      cv::Scalar(0.0f, 0.0f, 255.0f),
      cv::Scalar(255.0f, 255.0f, 0.0f),
      cv::Scalar(255.0f, 0.0f, 255.0f),
      cv::Scalar(0.0f, 255.0f, 255.0f),
      cv::Scalar(0.5f, 255.0f, 0.0f),
      cv::Scalar(0.5f, 0.0f, 255.0f),
      cv::Scalar(255.0f, 0.5f, 0.0f),
      cv::Scalar(255.0f, 0.0f, 0.5f),
  };
  cv::findContours(morphMat, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  hull.resize(contours.size());
  const int kMaxAreaSize = calibration_max_width_ * calibration_max_height_;
  const int kMinAreaSize = 20 * 20;
  areas.clear();
  for (int i = 0; i < contours.size(); i++) {
    cv::convexHull(contours[i], hull[i]);
    auto area = cv::contourArea(hull[i]);
    areas.push_back(area);

    if (area > kMaxAreaSize || area < kMinAreaSize) {
      continue;
    }

    auto minRect = cv::minAreaRect(contours[i]);
    std::vector<cv::Point2f> box(4);
    minRect.points(box.data());
    bounding_boxes.emplace_back(std::make_tuple(i, box));
  }

  // =========================================
  // 7. Calculate approximate hand positions
  // =========================================

  std::vector<cv::RotatedRect> hand_boxes;
  std::vector<cv::RotatedRect> secondary_hand_boxes;
  std::vector<std::vector<cv::Point2i>> final_contours{bounding_boxes.size()};
  std::vector<cv::RotatedRect> final_hand_boxes{bounding_boxes.size()};
  for (int i = 0; i < bounding_boxes.size(); i++) {
    auto bindex = std::get<0>(bounding_boxes[i]);
    auto &box = std::get<1>(bounding_boxes[i]);

    typedef cv::Point2f p2f;
    typedef std::vector<p2f> rect;
    typedef std::tuple<double, p2f, p2f, p2f, p2f, int> boxinfo;
    std::vector<boxinfo> side_lengths{
        {cv::norm(box[0] - box[1]), box[0], box[1], box[2], box[3], 0},
        {cv::norm(box[1] - box[2]), box[1], box[2], box[3], box[0], 1},
        {cv::norm(box[2] - box[3]), box[2], box[3], box[0], box[1], 2},
        {cv::norm(box[3] - box[0]), box[3], box[0], box[1], box[2], 3}
    };
    std::sort(side_lengths.begin(), side_lengths.end(), [](const boxinfo &b1, const boxinfo &b2) {
      double length1 = std::get<0>(b1);
      double length2 = std::get<0>(b2);
      return length1 < length2;
    });

    std::vector<std::tuple<rect, p2f, bool>> rects;
    for (int j = 0; j < 2; j++) {
      auto[length, x1, x2, x3, x4, _] = side_lengths[j];
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
      hand_boxes.emplace_back(rotatedRect1);
      secondary_hand_boxes.emplace_back(rotatedRect2);
      activeRect = rotatedRect1;
    } else {
      hand_boxes.emplace_back(rotatedRect2);
      secondary_hand_boxes.emplace_back(rotatedRect1);
      activeRect = rotatedRect2;
    }

    for (const auto& c : contours[bindex]) {
      if (PointInsideRotatedRect(c, activeRect)) {
        final_contours[i].emplace_back(c);
      }
    }

    if (!final_contours[i].empty()) {
      auto minRect = cv::minAreaRect(final_contours[i]);
      final_hand_boxes.emplace_back(minRect);
    }
  }

  // =========================================
  // 8. Segmentation Localization
  // =========================================

  // Update Tracker
  //multiTracker.updateClusters(boundingBoxes);
  multi_tracker_.UpdateCluster(hand_boxes);
  auto trackedClusters = multi_tracker_.GetClusters();

  // =========================================
  // 9. Drawing
  // =========================================

  // Draw Contours
  for (int i = 0; i < contours.size(); i++) {
    if (areas[i] < kMaxAreaSize && areas[i] > kMinAreaSize) {
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
    cv::putText(cv_color_frame, label.str(), cluster.rect.boundingRect().tl(), cv::FONT_HERSHEY_SIMPLEX, 1, color);
  }

  // Draw hand boxes
  for (const auto& handBox : hand_boxes) {
    std::vector<cv::Point2f> points(4);
    handBox.points(points.data());
    std::vector<std::vector<cv::Point2i>> polylines{{points[0], points[1], points[2], points[3]}};
    //cv::polylines(cvColorFrame, polylines, true, cv::Scalar(0.0f, 255.0f, 255.0f), 2, 8, 0);
  }
  for (const auto& handBox : secondary_hand_boxes) {
    std::vector<cv::Point2f> points(4);
    handBox.points(points.data());
    std::vector<std::vector<cv::Point2i>> polylines{{points[0], points[1], points[2], points[3]}};
    //cv::polylines(cvColorFrame, polylines, true, cv::Scalar(255.0f, 0.0f, 255.0f), 2, 8, 0);
  }

  // Draw final contours
  // std::cout << "finalContours: " << finalContours.size() << std::endl;
  for (const auto& finalContour : final_contours) {
    cv::polylines(cv_color_frame, finalContour, true, cv::Scalar(0.0f, 255.0f, 0.0f), 2, 8, 0);
  }

  // Draw final handboxes
  for (const auto& finalHandBox : final_hand_boxes) {
    std::vector<cv::Point2f> points(4);
    finalHandBox.points(points.data());
    std::vector<std::vector<cv::Point2i>> polylines{{points[0], points[1], points[2], points[3]}};
    cv::polylines(cv_color_frame, polylines, true, cv::Scalar(255.0f, 0.0f, 123.0f), 2, 8, 0);
  }

  // Draw calibration window
  auto rw1 = (width / 2) - (calibration_max_width_ / 2);
  auto rw2 = (width / 2) + (calibration_max_width_ / 2);
  auto rh1 = (height / 2) - (calibration_max_height_ / 2);
  auto rh2 = (height / 2) + (calibration_max_height_ / 2);
  cv::rectangle(cv_color_frame, cv::Rect(cv::Point(rw1, rh1), cv::Size(calibration_max_width_, calibration_max_height_)), cv::Scalar(127, 0, 255));

  // =========================================
  // 10. Finalize
  // =========================================

  prev_cv_color_frame_ = cv_color_frame;
  // prevCvDepthFrame = cvDepthFrame;

  //processedCvColorFrame = cvColorFrame;
  //processedCvColorFrame = contourColorFrame;
  //processedCvColorFrame = greyMat;
  //processedCvColorFrame = blurMat;
  //processedCvColorFrame = threshMat;
  //processedCvColorFrame = dilateMat;
  //processedCvColorFrame = morphMat;
  switch (properties.frame_step) {
    default:
    case 0: processed_cv_color_frame_ = cv_color_frame; break;
    case 1: processed_cv_color_frame_ = contour_color_frame; break;
    case 2: processed_cv_color_frame_ = greyMat; break;
    case 3: processed_cv_color_frame_ = blurMat; break;
    case 4: processed_cv_color_frame_ = threshMat; break;
    case 5: processed_cv_color_frame_ = morphMat; break;
  }
  processed_cv_depth_frame_ = cvDepthFrame;

  processed_rs_2_color_frame_ = other_frame;
  processed_rs_2_depth_frame_ = aligned_filtered_depth_frame;

  // 10. Save frames in mats in case we want to display/save them later
  frame_cv_color_frame_ = cv_color_frame;
  frame_contour_color_frame_ = contour_color_frame;
  frame_grey_mat_ = greyMat;
  frame_blur_mat_ = blurMat;
  frame_thresh_mat_ = threshMat;
  frame_morph_mat_ = morphMat;
  return true;
}

cv::Mat RealsenseCameraManager::GetCvColorFrame() {
  return processed_cv_color_frame_;
}

cv::Mat RealsenseCameraManager::GetCvDepthFrame() {
  return processed_cv_depth_frame_;
}

void RealsenseCameraManager::Calibrate(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame) {
  auto p_depth_frame = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
  auto p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

  int width = depth_frame.get_width();
  int height = depth_frame.get_height();

  float totalDepth = .0f;
  float maxDepth = std::numeric_limits<float>::min();

  // Save random pixel distances to calculate the homography plane afterwards
  std::random_device rd;
  std::mt19937 gen(rd());
  auto rw1 = (width / 2) - (calibration_max_width_ / 2);
  auto rw2 = (width / 2) + (calibration_max_width_ / 2);
  auto rh1 = (height / 2) - (calibration_max_height_ / 2);
  auto rh2 = (height / 2) + (calibration_max_height_ / 2);
  std::uniform_int_distribution<> distrw(rw1, rw2);
  std::uniform_int_distribution<> distrh(rh1, rh2);
  constexpr auto sample_count = 4;
  auto A_samples = cv::Mat(sample_count, 3, CV_64F);
  auto b_samples = cv::Mat(sample_count, 1, CV_64F);

  A_samples.at<double>(0, 0) = static_cast<double>(rw1);
  A_samples.at<double>(0, 1) = static_cast<double>(rh1);
  A_samples.at<double>(0, 2) = 1.0f;
  b_samples.at<double>(0) = depth_scale_ * static_cast<float>(p_depth_frame[rw1 + (rh1 * width)]);

  A_samples.at<double>(1, 0) = static_cast<double>(rw2);
  A_samples.at<double>(1, 1) = static_cast<double>(rh1);
  A_samples.at<double>(1, 2) = 1.0f;
  b_samples.at<double>(1) = depth_scale_ * static_cast<float>(p_depth_frame[rw2 + (rh1 * width)]);

  A_samples.at<double>(2, 0) = static_cast<double>(rw2);
  A_samples.at<double>(2, 1) = static_cast<double>(rh2);
  A_samples.at<double>(2, 2) = 1.0f;
  b_samples.at<double>(2) = depth_scale_ * static_cast<float>(p_depth_frame[rw2 + (rh2 * width)]);

  A_samples.at<double>(3, 0) = static_cast<double>(rw1);
  A_samples.at<double>(3, 1) = static_cast<double>(rh2);
  A_samples.at<double>(3, 2) = 1.0f;
  b_samples.at<double>(3) = depth_scale_ * static_cast<float>(p_depth_frame[rw1 + (rh2 * width)]);

  std::cout << "Calibration: A_samples=" << A_samples << "\n";
  std::cout << "Calibration: b_samples=" << b_samples << "\n";

  // Calculate fitted plane
  cv::Mat fitted_plane;
  auto ret = cv::solve(A_samples, b_samples, fitted_plane, cv::DECOMP_SVD);
  std::cout << "Calibration: fitted_plane=" << fitted_plane << "\n";

  // Define Z = a*X + b*Y + c
  auto project = [&](double x, double y) {
    auto a = fitted_plane.at<double>(0), b = fitted_plane.at<double>(1), c = fitted_plane.at<double>(2);
    return (a * x) + (b * y) + c;
  };

  // Get some samples (3 should be enough but the more the better I guess)
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
  std::cout << "Calibration: from=" << from << "\n";
  std::cout << "Calibration: to=" << to << "\n";
  auto H = cv::findHomography(from, to);
  std::cout << "Calibration: H=" << H << " (" << H.rows << "," << H.cols << ")\n";
  H_z_row_[0] = H.at<double>(2, 0);
  H_z_row_[1] = H.at<double>(2, 1);
  H_z_row_[2] = H.at<double>(2, 2);
  std::cout << "Calibration: H_z_row_=[" << H_z_row_[0] << "," << H_z_row_[1] << "," << H_z_row_[2] << "\n"; //," << H_zrow[3] << "]\n";

  // Get the center point of the new plane and add some small value. This is the threshold, everything
  // that is below that, i.e. under the table will be ignored
  int centerX = width / 2;
  int centerY = height / 2;
  auto centerZ = depth_scale_ * static_cast<float>(p_depth_frame[centerX + (centerY * width)]);
  std::cout << "Calibration: center=[" << centerX << "," << centerY << "," << centerZ << "]\n";
  calibrated_z_ = (H_z_row_[0] * centerX) + (H_z_row_[1] * centerY) + (H_z_row_[2] * centerZ);
  calibrated_z_ -= 0.05;
  std::cout << "Calibration: calibrated_z_=" << calibrated_z_ << "\n";

  is_calibrated_ = true;
}

void RealsenseCameraManager::Recalibrate() {
  is_calibrated_ = false;
}

void RealsenseCameraManager::ApplyThreshold(rs2::video_frame& other_frame, rs2::depth_frame& depth_frame, unsigned char color)
{
  auto p_depth_frame = reinterpret_cast<uint16_t*>(const_cast<void*>(depth_frame.get_data()));
  auto p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

  auto width = other_frame.get_width();
  auto height = other_frame.get_height();
  auto other_bpp = other_frame.get_bytes_per_pixel();

  auto zMax = calibrated_z_ + properties.z_culling_back;
  auto zMin = calibrated_z_ - properties.z_culling_front;
  zMin -= 0.25f;
//sd#pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
  for (int y = 0; y < height; y++)
  {
    auto depth_pixel_index = y * width;
    for (int x = 0; x < width; x++, ++depth_pixel_index)
    {
      // Original depth value
      auto z = depth_scale_ * static_cast<float>(p_depth_frame[depth_pixel_index]);
      // Altered depth value
      double pixels_distance = (H_z_row_[0] * x) + (H_z_row_[1] * y) + (H_z_row_[2] * z);

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

rs2::video_frame RealsenseCameraManager::GetRs2ColorFrame() {
  return processed_rs_2_color_frame_.as<rs2::video_frame>();
}

rs2::depth_frame RealsenseCameraManager::GetRs2DepthFrame() {
  return processed_rs_2_depth_frame_.as<rs2::depth_frame>();
}

float RealsenseCameraManager::GetDepthScale(const rs2::device& dev)
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

rs2_stream RealsenseCameraManager::FindStreamToAlign(const std::vector<rs2::stream_profile>& streams)
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

bool RealsenseCameraManager::ProfileChanged(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
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
  pipe_->stop();
}

cv::Mat RealsenseCameraManager::ConvertFrameToMat(const rs2::frame& f)
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
cv::Mat RealsenseCameraManager::ConvertDepthFrameToMetersMat(const rs2::depth_frame & f)
{
  cv::Mat dm = ConvertFrameToMat(f);
  dm.convertTo(dm, CV_64F);
  dm = dm * f.get_units();
  return dm;
}

void RealsenseCameraManager::Screenshot(int step, SCREENSHOT_FLAGS flags, const std::string& screenshot_path) {
  cv::Mat mat;
  switch (step) {
    default:
    case 0: mat = frame_cv_color_frame_; break;
    case 1: mat = frame_contour_color_frame_; break;
    case 2: mat = frame_grey_mat_; break;
    case 3: mat = frame_blur_mat_; break;
    case 4: mat = frame_thresh_mat_; break;
    case 5: mat = frame_morph_mat_; break;
  }

  if (flags == SAVE || flags == DISPLAY_SAVE) {
    std::stringstream imagePath;
#ifdef APPLE
    auto t = std::time(nullptr);
#else
    auto t = time(nullptr);
#endif
    imagePath << screenshot_path << t << ".png";
    std::cout << "Saved Screenshot of '" << GetFrameStepLabel(step) << "' to '" << imagePath.str() << "'";
    cv::imwrite(imagePath.str(), mat);
  }

  if (flags == DISPLAY || flags == DISPLAY_SAVE) {
    std::stringstream windowName;
    windowName << "Display '" << GetFrameStepLabel(step) << "'";
    cv::imshow(windowName.str(), mat);
    cv::waitKey(0);
    cv::destroyWindow(windowName.str());
  }
}

const char* RealsenseCameraManager::GetFrameStepLabel(int step) {
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

inline double RealsenseCameraManager::Shoelace(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3) {
  return std::fabs(0.5 * (((p2.x - p1.x) * (p3.y - p1.y)) - ((p3.x - p1.x) * (p2.y - p1.y))));
}

inline bool RealsenseCameraManager::PointInsideRotatedRect(const cv::Point2f& point, const cv::RotatedRect& rotated_rect) {
  cv::Point2f points[4];
  rotated_rect.points(points);
  const auto [rp1, rp2, rp3, rp4] = points;
  const auto area12p = Shoelace(rp1, rp2, point);
  const auto area23p = Shoelace(rp2, rp3, point);
  const auto area34p = Shoelace(rp3, rp4, point);
  const auto area41p = Shoelace(rp4, rp1, point);
  const auto areaPSum = area12p + area23p + area34p + area41p;
  const auto areaSum = Shoelace(rp1, rp2, rp3) + Shoelace(rp1, rp4, rp3);
  return areaPSum <= areaSum;
}