#ifndef APPLE

#include <Kinect.h>
#include "KinectManager_Windows.h"
#include "InputRecorder.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <QtGui/QOpenGLBuffer>
#include <QtCore/QDebug>
#include <iostream>
#include <fstream>
#include <chrono>

#include <filesystem>
#include <string>
//#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>

int KinectManager::initialize() {
    if (FAILED(GetDefaultKinectSensor(&sensor)) || !sensor) {
        return S_FALSE;
    }

    sensor->get_CoordinateMapper(&mapper);
    sensor->Open();
    sensor->OpenMultiSourceFrameReader(
            FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color,
            &reader
    );

	loadedKinect = LoadingStatus::KINECT;
    return S_OK;
}

int KinectManager::initializeFromFile(std::string path) {
	if (FAILED(GetDefaultKinectSensor(&sensor)) || !sensor) {
		return S_FALSE;
	}

	sensor->get_CoordinateMapper(&mapper);
	reader = new InputRecorder(path);

	loadedKinect = LoadingStatus::FILE;
	return S_OK;
}

void KinectManager::terminate() {
	if (loadedKinect == LoadingStatus::KINECT) {
		sensor->Close();
	}
	else if (loadedKinect == LoadingStatus::FILE) {
		delete reader;
	}
	loadedKinect = LoadingStatus::UNLOADED;
}

long KinectManager::AcquireLatestFrame() {
	assert(loadedKinect);

	//PointF *tableEntries;
	//UINT32 tableEntryCount;
	//auto a1 = mapper->GetDepthFrameToCameraSpaceTable(&tableEntryCount, &tableEntries);
	//CameraIntrinsics intrinsics;
	//auto a2 = mapper->GetDepthCameraIntrinsics(&intrinsics);
	//sensor;

	HRESULT result;

	if (loadedKinect == LoadingStatus::KINECT) {
		result = reader->AcquireLatestFrame(&lastFrame);
		if (result == S_OK) {
			frameCount++;
			//qDebug() << "Frame count is now " << frameCount;
		}
	}

	else if (loadedKinect == LoadingStatus::FILE) {
		result = reader->AcquireLatestFrame(&lastFrame);
		if (result == S_OK) {
			frameCount++;
			//qDebug() << "Frame count is now " << frameCount;
		}
	}

	if (result != S_OK) {
		return -1;
	}

	return result;
}

void KinectManager::ReleaseLatestFrame() {
	assert(loadedKinect);
}


void KinectManager::jumpToFrame(int frame) {
	if (loadedKinect != LoadingStatus::FILE) {
		return;
	}

	auto ir = dynamic_cast<InputRecorder*>(reader);
	ir->jumpToFrame(frame);

}

const int KinectManager::getDepthSize() {
	assert(loadedKinect);

	return WIDTH * HEIGHT * 3 * sizeof(float);
}

const int KinectManager::getRgbSize() {
	assert(loadedKinect);

	return WIDTH * HEIGHT * 3 * sizeof(float);
}

void KinectManager::writeDepthData(void *dest) {
	assert(loadedKinect);

	if (dest == NULL) {
		return;
	}

    IDepthFrame* depthframe;
    IDepthFrameReference* frameref = NULL;

	lastFrame->get_DepthFrameReference(&frameref);
    frameref->AcquireFrame(&depthframe);
    if (frameref) {
        frameref->Release();
    }

    if (!depthframe) {
        return;
    }

	currentDepthBuf = nullptr;

    // Get data from frame
    unsigned int sz;
    HRESULT result;
	if (loadedKinect == LoadingStatus::KINECT) {
		result = depthframe->AccessUnderlyingBuffer(&sz, &currentDepthBuf);
		result = mapper->MapDepthFrameToCameraSpace(WIDTH * HEIGHT, currentDepthBuf, WIDTH * HEIGHT, depth2xyz);
	}
	else {
		auto df = dynamic_cast<InputRecorder::DepthFrame*>(depthframe);
		result = df->copyDepthData(sizeof(depth2rgb), reinterpret_cast<BYTE*>(depth2rgb), sizeof(depth2xyz), reinterpret_cast<BYTE*>(depth2xyz));
		sz = WIDTH * HEIGHT;
	}

    float* fdest = (float*)dest;
    for (unsigned int i = 0; i < sz; i++) {
        *fdest++ = depth2xyz[i].X;
        *fdest++ = depth2xyz[i].Y;
        *fdest++ = depth2xyz[i].Z;
    }

	if (loadedKinect == LoadingStatus::KINECT) {
		mapper->MapDepthFrameToColorSpace(WIDTH * HEIGHT, currentDepthBuf, WIDTH * HEIGHT, depth2rgb);
	}

	result = mapper->MapDepthFrameToCameraSpace(WIDTH * HEIGHT, currentDepthBuf, WIDTH * HEIGHT, color2camera);

    //glBuffer->unmap();	
	
	if (depthframe) {
		depthframe->Release();
	}
}

void KinectManager::writeRgbData(void *dest) {
	assert(loadedKinect);

	if (dest == NULL) {
		return;
	}

    IColorFrame* colorframe;
    IColorFrameReference* frameref = NULL;

	lastFrame->get_ColorFrameReference(&frameref);
    frameref->AcquireFrame(&colorframe);
    if (frameref) {
        frameref->Release();
    }

    if (!colorframe) {
        return;
    }

    /*glBuffer->allocate(WIDTH * HEIGHT * 3 * sizeof(float));
    auto dest = glBuffer->mapRange(0, WIDTH * HEIGHT * 3 * sizeof(float), QOpenGLBuffer::RangeInvalidateBuffer | QOpenGLBuffer::RangeWrite);
    if (dest == NULL) {
        return;
    }*/

    colorframe->CopyConvertedFrameDataToArray(COLORWIDTH * COLORHEIGHT * 4, rgbimage, ColorImageFormat_Rgba);

    float* fdest = (float*)dest;
    for (int i = 0; i < WIDTH * HEIGHT; i++) {
        ColorSpacePoint p = depth2rgb[i];
        // Check if color pixel coordinates are in bounds
        if (p.X < 0 || p.Y < 0 || p.X > COLORWIDTH || p.Y > COLORHEIGHT) {
            *fdest++ = 0;
            *fdest++ = 0;
            *fdest++ = 0;
        }
        else {
            int idx = (int)p.X + COLORWIDTH * (int)p.Y;
            *fdest++ = rgbimage[4 * idx + 0] / 255.;
            *fdest++ = rgbimage[4 * idx + 1] / 255.;
            *fdest++ = rgbimage[4 * idx + 2] / 255.;
			// Don't copy alpha channel
        }
    }

    //glBuffer->unmap();

	if (colorframe) {
		colorframe->Release();
	}
}

void KinectManager::saveRGBImage(const std::string& path) {
	assert(loadedKinect);

	saveRGBImage(path, rgbimage, COLORWIDTH, COLORHEIGHT);
}

void KinectManager::saveRGBImage(const std::string& path, unsigned char *input, int width, int height) {
	assert(loadedKinect);

    cv::Mat frameRGB(height, width, CV_8UC4, input);
    cv::Mat frameBGRA(height, width, CV_8UC4);
    cv::cvtColor(frameRGB, frameBGRA, cv::COLOR_RGBA2BGRA);
    cv::Mat flippedFrame;
    cv::flip(frameBGRA, flippedFrame, 1);
    cv::imwrite(path, flippedFrame);
}

int KinectManager::getCurrentFrame() const {
	if (loadedKinect != LoadingStatus::FILE) {
		return -1;
	}

	auto ir = dynamic_cast<InputRecorder*>(reader);
	return ir->getCurrentFrame();
}

#endif