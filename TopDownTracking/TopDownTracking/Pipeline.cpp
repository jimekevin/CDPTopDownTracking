#include "Pipeline.h"

void Pipeline::enableTasks(std::vector<bool> &enabledTasks) {
	for (int i = 0; i < tasks.size(); ++i) {
		tasks[i]->enabled = enabledTasks[i];
	}
}

void Pipeline::process(cv::Mat& depthMat, cv::Mat& rgbMat) {
	for (const auto &task : tasks) {
		if (!task->enabled) {
			continue;
		}
		task->apply(depthMat, rgbMat);
	}
}

size_t Pipeline::addProcessingTask(Task *filter) {
	tasks.push_back(filter);
	return tasks.size() - 1;
}

void Pipeline::removeProcessingTask(size_t index) {
	if (index >= 0 && index < tasks.size()) {
		tasks.erase(tasks.begin() + index);
	}
}