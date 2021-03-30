#ifndef APPLE

#ifndef KINECT_MANAGER_WINDOWS_H
#define KINECT_MANAGER_WINDOWS_H

#include <Kinect.h>
#include <opencv2/imgproc.hpp>
#include <QtGui/QOpenGLBuffer>
#include <thread>
#include <fstream>
#include <boost/iostreams/filtering_stream.hpp>

class KinectManager {
public:
  const static int DELAY_MS = 16; //33;
  const static int WIDTH = 512;
  const static int HEIGHT = 424;
  const static int COLORWIDTH = 1920;
  const static int COLORHEIGHT = 1080;

  IMultiSourceFrameReader* reader = nullptr;

  UINT16* currentDepthBuf = nullptr; 
  // 3 BELOW MUST NOT BE REORDERED!
  ColorSpacePoint depth2rgb[WIDTH * HEIGHT];  // Maps depth pixels to rgb pixels
  CameraSpacePoint depth2xyz[WIDTH * HEIGHT];
  unsigned char rgbimage[COLORWIDTH * COLORHEIGHT * 4]; // RGBA 0-255
  CameraSpacePoint color2camera[WIDTH * HEIGHT];

  int frameCount = 0; 

  enum LoadingStatus { UNLOADED, KINECT, FILE };
  LoadingStatus loadedKinect = UNLOADED;

private:

  KinectManager() {} // Disallow instantiation outside of the class.

  IKinectSensor *sensor;
  ICoordinateMapper *mapper; // Converts between depth, color, and 3d coordinates

  IColorFrameReader *colorFrameReader;
  IDepthFrameReader *depthFrameReader;

  IMultiSourceFrame *lastFrame = nullptr;

  bool videoRecording = false;

public:
  KinectManager(const KinectManager&) = delete;
  KinectManager& operator=(const KinectManager &) = delete;
  KinectManager(KinectManager &&) = delete;
  KinectManager & operator=(KinectManager &&) = delete;

  static auto& instance() {
      static KinectManager test;
      return test;
  }

  int initialize();
  int initializeFromFile(std::string path);
  void terminate();
  
  long AcquireLatestFrame();
  void ReleaseLatestFrame();
  void jumpToFrame(int frame);

  const int getDepthSize();
  const int getRgbSize();

  void writeDepthData(void *dest);
  void writeRgbData(void *dest);

  void saveRGBImage(const std::string& path);
  void saveRGBImage(const std::string& path, unsigned char *input, int width, int height);

  int getCurrentFrame() const;
};

#endif // KINECT_MANAGER_WINDOWS_H

#endif