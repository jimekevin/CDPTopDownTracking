#ifndef KINECT_MANAGER_H
#define KINECT_MANAGER_H

#ifdef APPLE

#include "KinectManager_MacOS.h"

#include <iostream>
#include <chrono>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <QtGui/QOpenGLBuffer>
#include <utility>
#include <fstream>

//typedef struct { float X, Y; } ColorSpacePoint;
//typedef struct { float X, Y, Z; } CameraSpacePoint;

// https://openkinect.github.io/libfreenect2/

int KinectManager::initialize(std::string videoSource) {
    std::cout << "Test" << std::endl;
    return 0;
}

int KinectManager::initialize() {
    libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));

    if(freenect2.enumerateDevices() == 0) {
        std::cout << "No device connected!" << std::endl;
        return -1;
    }
    if (serial.empty()) {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }

    // TODO: switch pipelines?
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
    pipeline = new libfreenect2::CpuPacketPipeline();
#endif
    dev = freenect2.openDevice(serial, pipeline);
    if(dev == nullptr) {
        std::cout << "Failure opening device!" << std::endl;
        return -1;
    }

    int types = libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    listener = new libfreenect2::SyncMultiFrameListener(types);

    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);

    // Start the device
    if (!dev->start())  {
        return -1;
    }
    /*if (!dev->startStreams(true, true)) {
        return -1;
    }*/

    startedStreams = true;
    std::cout << "Device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "Device firmware: " << dev->getFirmwareVersion() << std::endl;

    return 0;
}

void KinectManager::terminate() {
    if (dev != nullptr) {
        dev->stop();
        dev->close();
    }
}

long KinectManager::AcquireLatestFrame() {
    if (!startedStreams) {
        return -1;
    }
    if (!listener->waitForNewFrame(frames, 10*1000)) {
        return -1;
    }
    rgbFrame = frames[libfreenect2::Frame::Color];
    irFrame = frames[libfreenect2::Frame::Ir];
    depthFrame = frames[libfreenect2::Frame::Depth];

    // Map color images onto depth images
    auto registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    registration->apply(rgbFrame, depthFrame, &undistorted, &registered);

    return 0;
}

void KinectManager::ReleaseLatestFrame() {
    if (startedStreams) {
        listener->release(frames);
    }
}

const int KinectManager::getDepthSize() {
    return WIDTH * HEIGHT * 3 * sizeof(float);
}

const int KinectManager::getRgbSize() {
    return WIDTH * HEIGHT * 3 * sizeof(float);
}

struct DepthPoint {
  float x;
  float y;
  float z;
};

void KinectManager::writeDepthData(void *dest) {
    if (dest == nullptr) {
        return;
    }

    size_t sz = depthFrame->width * depthFrame->height;
    float maxZ = -1.0f;
    auto fdest = (float*)dest;
    for (unsigned int i = 0; i < sz; i++) {
//        auto dp = reinterpret_cast<DepthPoint&>(depthFrame->data[4 * i]);
//        auto x = dp.x;
//        auto y = dp.y;
//        auto z = dp.z;
//        auto x2 = (float)depthFrame->data[4 * i + 0];
//        auto y2 = (float)depthFrame->data[4 * i + 1];
//        auto z2 = (float)depthFrame->data[4 * i + 2];
        //*fdest++ = dp.x / 1000.0f; //(float)depthFrame->data[4 * i + 0] / 100.0f;
        //*fdest++ = dp.y / 1000.0f; //(float)depthFrame->data[4 * i + 1] / 100.0f;
        //*fdest++ = dp.z / 1000.0f; //(float)depthFrame->data[4 * i + 2] / 100.0f;
        //*fdest++ = depthFrame->data[4 * i + 0] / 1000.0f;
        //*fdest++ = depthFrame->data[4 * i + 1] / 1000.0f;
        //*fdest++ = depthFrame->data[4 * i + 2] / 1000.0f;
        auto x = ((float)(i % depthFrame->width) / depthFrame->width * 2.0f) - 1.0f;
        auto y = 1.0f - ((float)((int)(i / depthFrame->width)) / depthFrame->height * 2.0f);
        auto z = reinterpret_cast<float&>(registered.data[depthFrame->bytes_per_pixel * i]) / 1000.f;
        *fdest++ = x;
        *fdest++ = y;
        *fdest++ = z;
        maxZ = z > maxZ ? z : maxZ;
    }
    auto test = 1;
}

void KinectManager::writeRgbData(void *dest) {
    if (dest == nullptr) {
        return;
    }

    /*if (rgbFrame == nullptr || depthFrame == nullptr) {
        return;
    }*/

    auto fdest = (float*)dest;
    for (int i = 0; i < WIDTH * HEIGHT; i++) {
        *fdest++ = 1.0f;
        *fdest++ = 0.0f;
        *fdest++ = 0.0f;
        //*fdest++ = registered.data[4 * i + 2];
        //*fdest++ = registered.data[4 * i + 1];
        //*fdest++ = registered.data[4 * i + 0];
        //ColorSpacePoint* p = reinterpret_cast<ColorSpacePoint*>(&registered.data[i]);
        // Check if color pixel coordinates are in bounds
//        if (p->X < 0 || p->Y < 0 || p->X > COLORWIDTH || p->Y > COLORHEIGHT) {
//            *fdest++ = 0;
//            *fdest++ = 0;
//            *fdest++ = 0;
//        }
//        else {
//            int idx = (int)p.X + COLORWIDTH * (int)p.Y;
//            *fdest++ = rgbimage[4 * idx + 0] / 255.;
//            *fdest++ = rgbimage[4 * idx + 1] / 255.;
//            *fdest++ = rgbimage[4 * idx + 2] / 255.;
        //}
        // Don't copy alpha channel
    }
}

void KinectManager::saveRGBImage(const std::string& path) {
    saveRGBImage(path, rgbimage, COLORWIDTH, COLORHEIGHT);
}

void KinectManager::saveRGBImage(const std::string& path, unsigned char *input, int width, int height) {
    cv::Mat frameRGB(height, width, CV_8UC4, input);
    cv::Mat frameBGRA(height, width, CV_8UC4);
    cv::cvtColor(frameRGB, frameBGRA, cv::COLOR_RGBA2BGRA);
    cv::Mat flippedFrame;
    cv::flip(frameBGRA, flippedFrame, 1);
    cv::imwrite(path, flippedFrame);
}

void KinectManager::startVideoRecording(const std::string& path) {
    return;
    /*
    if (videoRecording) {
        return;
    }

    // Thread dispatch

    std::ofstream ofs(path, std::ofstream::out | std::ofstream::trunc);
    int count = 0;
    while (videoRecording) {
        if (count >= frameCount) {
            continue;
        }

        ofs.write((char*)depth2xyz, WIDTH * HEIGHT * 4);
        ofs.write((char*)rgbimage, COLORWIDTH * COLORHEIGHT * 4);
        std::cout << "Wrote frame " << frameCount << std::endl;
        count++;
    }
    ofs.close();
     */
}

void KinectManager::stopVideoRecording() {
    if (!videoRecording) {
        return;
    }

    videoRecording = false;
}

int KinectManager::loadVideo(std::string videoSource) {
	std::cout << "Not implemented" << std::endl;
	return 0;
}

#endif // APPLE

#endif // KINECT_MANAGER_H