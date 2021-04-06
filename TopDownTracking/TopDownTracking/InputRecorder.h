#ifndef INPUT_RECORDER_H
#define INPUT_RECORDER_H

#ifndef APPLE

#include <Kinect.h>

#include <iostream>
#include <fstream>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>

// IUnknown Implementation
#define IMPLEMENT_IUNKNOWN_WITHOUT_RELEASE \
	HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void __RPC_FAR *__RPC_FAR *ppvObject) { return S_FALSE; } \
	ULONG STDMETHODCALLTYPE AddRef(void) { return 0; }
#define IMPLEMENT_IUNKNOWN \
	IMPLEMENT_IUNKNOWN_WITHOUT_RELEASE \
	ULONG STDMETHODCALLTYPE Release(void) { return 0; } 

class InputRecorder : public IMultiSourceFrameReader
{
public:

	struct RecorderInfo {
		int depthWidth = 512;
		int depthHeight = 424;
		int colorWidth = 1920;
		int colorHeight = 1080;

		int depthSize() const {
			return depthWidth * depthHeight;
		}
		int depthByteSize() const {
			return depthSize() * sizeof(UINT16);
		}
		int colorSize() const {
			return colorWidth * colorHeight;
		}
		int colorByteSize() const {
			return colorWidth * colorHeight * sizeof(unsigned char) * 4;
		}

		int depth2rgbByteSize() const {
			return depthWidth * depthHeight * sizeof(ColorSpacePoint);
		}
		int depth2xyzByteSize() const {
			return depthWidth * depthHeight * sizeof(CameraSpacePoint);
		}
		int rgbimageByteSize() const {
			return colorWidth * colorHeight * 4 * sizeof(unsigned char);
		}
	};

	class ColorFrame : public IColorFrame {
	private:
		RecorderInfo recInfo;
		char *rgbimage;

	public:
		ColorFrame(char *rgbimage, RecorderInfo &SrecInfo);

		ULONG STDMETHODCALLTYPE Release();
		HRESULT STDMETHODCALLTYPE CopyConvertedFrameDataToArray(UINT capacity, /* [annotation][size_is][out] */ _Out_writes_all_(capacity) BYTE *frameData, ColorImageFormat colorFormat);

		HRESULT STDMETHODCALLTYPE get_RawColorImageFormat(ColorImageFormat *rawColorImageFormat) { return S_FALSE; }
		HRESULT STDMETHODCALLTYPE get_FrameDescription(IFrameDescription **rawFrameDescription) { return S_FALSE; }
		HRESULT STDMETHODCALLTYPE CopyRawFrameDataToArray(UINT capacity, BYTE *frameData) { return S_FALSE; }
		HRESULT STDMETHODCALLTYPE AccessRawUnderlyingBuffer(UINT *capacity, BYTE **buffer) { return S_FALSE; }
		HRESULT STDMETHODCALLTYPE CreateFrameDescription(ColorImageFormat format, IFrameDescription **frameDescription) { return S_FALSE; }
		HRESULT STDMETHODCALLTYPE get_ColorCameraSettings(IColorCameraSettings **colorCameraSettings) { return S_FALSE; }
		HRESULT STDMETHODCALLTYPE get_RelativeTime(TIMESPAN *relativeTime) { return S_FALSE; }
		HRESULT STDMETHODCALLTYPE get_ColorFrameSource(IColorFrameSource **colorFrameSource) { return S_FALSE; }

		IMPLEMENT_IUNKNOWN_WITHOUT_RELEASE
	};
	class ColorFrameReference : public IColorFrameReference {
	private:
		RecorderInfo recInfo;
		char *rgbimage;

	public:
		ColorFrameReference(char *rgbimage, RecorderInfo &recInfo);

		HRESULT STDMETHODCALLTYPE AcquireFrame(IColorFrame **colorFrame);

		HRESULT STDMETHODCALLTYPE get_RelativeTime(TIMESPAN *relativeTime) { return S_FALSE; }

		IMPLEMENT_IUNKNOWN
	};


	class DepthFrame : public IDepthFrame {
	private:
		RecorderInfo recInfo;
		char *depth2rgb;
		char *depth2xyz;

	public:
		DepthFrame(char *depth2rgb, char *depth2xyz, RecorderInfo &recInfo);

		HRESULT copyDepthData(UINT capacityRgb, BYTE *rgb, UINT capacityXyz, BYTE *xyz);

		ULONG STDMETHODCALLTYPE Release();
		HRESULT STDMETHODCALLTYPE AccessUnderlyingBuffer(UINT *capacity, UINT16 **buffer);

		HRESULT STDMETHODCALLTYPE CopyFrameDataToArray(UINT capacity, UINT16 *frameData) { return S_FALSE; }
		HRESULT STDMETHODCALLTYPE get_FrameDescription(IFrameDescription **frameDescription) { return S_FALSE; }
		HRESULT STDMETHODCALLTYPE get_RelativeTime(TIMESPAN *relativeTime) { return S_FALSE; }
		HRESULT STDMETHODCALLTYPE get_DepthFrameSource(IDepthFrameSource **depthFrameSource) { return S_FALSE; }
		HRESULT STDMETHODCALLTYPE get_DepthMinReliableDistance(USHORT *depthMinReliableDistance) { return S_FALSE; }
		HRESULT STDMETHODCALLTYPE get_DepthMaxReliableDistance(USHORT *depthMaxReliableDistance) { return S_FALSE; }

		IMPLEMENT_IUNKNOWN_WITHOUT_RELEASE
	};
	class DepthFrameReference : public IDepthFrameReference {
	private:
		RecorderInfo recInfo;
		char *depth2rgb;
		char *depth2xyz;

	public:
		DepthFrameReference(char *depth2rgb, char *depth2xyz, RecorderInfo &recInfo);

		HRESULT STDMETHODCALLTYPE AcquireFrame(IDepthFrame **depthFrame);

		HRESULT STDMETHODCALLTYPE get_RelativeTime(TIMESPAN *relativeTime) { return S_FALSE; }

		IMPLEMENT_IUNKNOWN
	};


	struct MultiSourceFrame : public IMultiSourceFrame {
	private:
		RecorderInfo recInfo;
		char *depth2rgb = nullptr;
		char *depth2xyz = nullptr;
		char *rgbimage = nullptr;

	public:
		MultiSourceFrame(char *depth2rgb, char *depth2xyz, char *rgbimage, RecorderInfo &recInfo);

		HRESULT STDMETHODCALLTYPE get_ColorFrameReference(_COM_Outptr_  IColorFrameReference **colorFrameReference);
		HRESULT STDMETHODCALLTYPE get_DepthFrameReference(_COM_Outptr_  IDepthFrameReference **depthFrameReference);

		HRESULT STDMETHODCALLTYPE get_BodyFrameReference(_COM_Outptr_  IBodyFrameReference **bodyFrameReference) { return S_FALSE; }
		HRESULT STDMETHODCALLTYPE get_BodyIndexFrameReference(_COM_Outptr_  IBodyIndexFrameReference **bodyIndexFrameReference) { return S_FALSE; }
		HRESULT STDMETHODCALLTYPE get_InfraredFrameReference(_COM_Outptr_  IInfraredFrameReference **infraredFrameReference) { return S_FALSE; }
		HRESULT STDMETHODCALLTYPE get_LongExposureInfraredFrameReference(_COM_Outptr_  ILongExposureInfraredFrameReference **longExposureInfraredFrameReference) { return S_FALSE; }

		IMPLEMENT_IUNKNOWN
	};

private:
	std::ifstream ifs;
	//boost::iostreams::filtering_istream in; // input file stream
	boost::iostreams::filtering_stream<boost::iostreams::input_seekable> in;
	int currentFrame = 0;
	int nextFrame = -1;

	RecorderInfo recInfo;
	int headerOffset = 0;

	char *depth2rgb = nullptr;
	char *depth2xyz = nullptr;
	char *rgbimage = nullptr;

	std::string path;

public:
	InputRecorder(std::string path);
	~InputRecorder();

	void jumpToFrame(int frame);
	int getCurrentFrame() const;

	HRESULT STDMETHODCALLTYPE AcquireLatestFrame(IMultiSourceFrame **multiSourceFrame);

	// IMultiSourceFrameReader
	HRESULT STDMETHODCALLTYPE SubscribeMultiSourceFrameArrived(WAITABLE_HANDLE *waitableHandle) { return S_FALSE; }
	HRESULT STDMETHODCALLTYPE UnsubscribeMultiSourceFrameArrived(WAITABLE_HANDLE waitableHandle) { return S_FALSE; }
	HRESULT STDMETHODCALLTYPE GetMultiSourceFrameArrivedEventData(WAITABLE_HANDLE waitableHandle, IMultiSourceFrameArrivedEventArgs **eventData) { return S_FALSE; }
	HRESULT STDMETHODCALLTYPE get_FrameSourceTypes(DWORD *enabledFrameSourceTypes) { return S_FALSE; }
	HRESULT STDMETHODCALLTYPE get_IsPaused(BOOLEAN *isPaused) { return S_FALSE; }
	HRESULT STDMETHODCALLTYPE put_IsPaused(BOOLEAN isPaused) { return S_FALSE; }
	HRESULT STDMETHODCALLTYPE get_KinectSensor(IKinectSensor **sensor) { return S_FALSE; }

	IMPLEMENT_IUNKNOWN
};

#endif // APPLE

#endif // INPUT_RECORDER_H
