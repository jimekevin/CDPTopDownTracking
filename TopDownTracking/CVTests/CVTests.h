#ifndef CVTESTS_CVTESTS_H
#define CVTESTS_CVTESTS_H

#include <librealsense2/rs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

namespace CVTests
{
	class Processor
	{
		Processor();
		~Processor();

		void process(cv::InputOutputArray depthMat, cv::InputOutputArray colorMat);
	};
};

#endif // CVTESTS_CVTESTS_H