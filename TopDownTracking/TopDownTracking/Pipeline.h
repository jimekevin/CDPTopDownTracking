#ifndef PIPELINE_H
#define PIPELINE_H

#include <vector>
#include <opencv2/imgproc.hpp>

class Pipeline
{
public:
	class Task
	{
	public:
		bool enabled = true;

		virtual void apply(cv::Mat& depthMat, cv::Mat& rgbMat) {}
	};

	void enableTasks(std::vector<bool> &enabledTasks);
	void process(cv::Mat& depthMat, cv::Mat& rgbMat);

	size_t addProcessingTask(Task *task);
	void removeProcessingTask(size_t index);

private:
	std::vector<Task*> tasks;
};

#endif // PIPELINE_H
