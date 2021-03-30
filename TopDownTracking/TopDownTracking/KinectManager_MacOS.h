#ifndef KINECTMANAGER_MACOS_H
#define KINECTMANAGER_MACOS_H

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <QtGui/QOpenGLBuffer>

class KinectManager {

public:
  const static int DELAY_MS = 33;
  const static int WIDTH = 512;
  const static int HEIGHT = 424;
  const static int COLORWIDTH = 1920;
  const static int COLORHEIGHT = 1080;

  unsigned char rgbimage[COLORWIDTH * COLORHEIGHT * 4]{};

private:
  int frameCount = 0;
  long long lastFrameFetch = 0;
  bool videoRecording = false;

  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = nullptr;
  libfreenect2::PacketPipeline *pipeline = nullptr;
  libfreenect2::SyncMultiFrameListener *listener = nullptr;
  libfreenect2::FrameMap frames;

  libfreenect2::Frame *rgbFrame = nullptr;
  libfreenect2::Frame *irFrame = nullptr;
  libfreenect2::Frame *depthFrame = nullptr;

  libfreenect2::Frame undistorted = libfreenect2::Frame(WIDTH, HEIGHT, 4);
  libfreenect2::Frame registered = libfreenect2::Frame(WIDTH, HEIGHT, 4);

  std::string serial;
  bool startedStreams = false;

  KinectManager() = default; // Disallow instantiation outside of the class.

public:

  KinectManager(const KinectManager&) = delete;
  KinectManager& operator=(const KinectManager &) = delete;
  KinectManager(KinectManager &&) = delete;
  KinectManager & operator=(KinectManager &&) = delete;

  static auto& instance() {
      static KinectManager i;
      return i;
  }

  int initialize();
  int initialize(std::string videoSource);
  void terminate();

  long AcquireLatestFrame();
  void ReleaseLatestFrame();

  const int getDepthSize();
  const int getRgbSize();

  void writeDepthData(void *dest);
  void writeRgbData(void *dest);

  void saveRGBImage(const std::string& path);
  void saveRGBImage(const std::string& path, unsigned char *input, int width, int height);
  void startVideoRecording(const std::string& path);
  void stopVideoRecording();
  int loadVideo(std::string videoSource);
};

#endif // KINECTMANAGER_MACOS_H
