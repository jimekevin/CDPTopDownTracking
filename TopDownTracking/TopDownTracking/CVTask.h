#ifndef TOPDOWNTRACKING_CVTASK_H
#define TOPDOWNTRACKING_CVTASK_H

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

class CVTask {
public:
  virtual void process(cv::InputOutputArray depthMat, cv::InputOutputArray colorMat) = 0;
};

#endif //TOPDOWNTRACKING_CVTASK_H
