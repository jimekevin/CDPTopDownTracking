#include "ThresholdFilter.h"

#include <cmath>
#include <QtCore/QDebug>

ThresholdFilter::ThresholdFilter(float minX, float maxX, float minY, float maxY, float minZ, float maxZ)
        : minX(minX), maxX(maxX), minY(minY), maxY(maxY), minZ(minZ), maxZ(maxZ)
{
}

void ThresholdFilter::updateMinMax(float minX, float maxX, float minY, float maxY, float minZ, float maxZ) {
    this->minX = minX != std::numeric_limits<double>::infinity() ? minX : this->minX;
    this->maxX = maxX != std::numeric_limits<double>::infinity() ? maxX : this->maxX;
    this->minY = minY != std::numeric_limits<double>::infinity() ? minY : this->minY;
    this->maxY = maxY != std::numeric_limits<double>::infinity() ? maxY : this->maxY;
    this->minZ = minZ != std::numeric_limits<double>::infinity() ? minZ : this->minZ;
    this->maxZ = maxZ != std::numeric_limits<double>::infinity() ? maxZ : this->maxZ;
}

//void ThresholdFilter::apply(cv::Mat& depthMat, cv::Mat& colorMat) {
void ThresholdFilter::process(cv::InputOutputArray depthMat, cv::InputOutputArray colorMat) {
    typedef cv::Point3_<uint8_t> RGB;
    cv::uint16_t minDepth = 38527;
    cv::uint16_t maxDepth = 0;
    colorMat.getMatRef().forEach<RGB>([&](RGB &p, const int *position) -> void { // p is RGB
      //auto depth = depthMat.getMatRef().at<double>(position[0], position[1]);
      auto depth = depthMat.getMatRef().ptr<double>(position[0])[position[1]]; // is .ptr faster than .at?
      //minDepth = minDepth < depth ? minDepth : depth;
      //maxDepth = maxDepth > depth ? maxDepth : depth;
      if (depth >= 2.0f) {
          p.x = 255;
          p.y = 0;
          p.z = 0;
      } else if (depth < 0.0f) {
        p.x = 0;
        p.y = 255;
        p.z = 0;
      }
      //qDebug() << depth << "; " << static_cast<float>(depth) << "; (" << position[0] << ", " << position[1] << ", " << position[2] << ")";
    });

    return;

    // This throws access exceptions ...
    //#pragma omp parallel
    for (int y = 0; y < depthMat.rows(); y++) {
        auto depthRow = depthMat.getMat().ptr<cv::Vec3f>(y);
        auto colorRow = colorMat.getMat().ptr<cv::Vec3f>(y);
        for (int x = 0; x < depthMat.cols(); x++) {
            auto &depth = depthRow[x];
            auto &rgb = colorRow[x];
            auto rows1 = depthMat.rows();
            auto cols1 = depthMat.cols();
            auto rows2 = colorMat.rows();
            auto cols2 = colorMat.cols();
            if (depth[0] < minX || depth[0] > maxX ||
                depth[1] < minY || depth[1] > maxY ||
                depth[2] < minZ || depth[2] > maxZ
                    ) {
                depthRow[x][0] = 0.0f;
                depthRow[x][1] = 0.0f;
                depthRow[x][2] = 0.0f;

                auto colorCol = colorRow[x];
                colorRow[x][0] = 1.0f;
                colorRow[x][1] = 1.0f;
                colorRow[x][2] = 1.0f;
            }
        }
    }
    /*for (int y = 0; y < colorMat->rows; y++) {
    for (int x = 0; x < colorMat->cols; x++) {
    auto &color = colorMat->at<cv::Vec3f>(y, x);
    color[0] = 1.0f;
    color[1] = 0.5f;
    color[2] = 0.1f;
    colorMat->at<cv::Vec3f>(cv::Point(x,y)) = color;
    }
    }*/
}
