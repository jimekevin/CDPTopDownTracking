#include "InputRecorder.h"

#include <QtCore/QtDebug>
#include <chrono>

InputRecorder::ColorFrame::ColorFrame(char *rgbimage, RecorderInfo &recInfo)
	: rgbimage(rgbimage), recInfo(recInfo)
{
}

ULONG STDMETHODCALLTYPE InputRecorder::ColorFrame::Release() {
	//delete rgbimage;
	return 0;
}

HRESULT STDMETHODCALLTYPE InputRecorder::ColorFrame::CopyConvertedFrameDataToArray(UINT capacity, /* [annotation][size_is][out] */ _Out_writes_all_(capacity) BYTE *frameData, ColorImageFormat colorFormat) {
	memcpy(frameData, rgbimage, capacity);
	return S_OK;
}


InputRecorder::ColorFrameReference::ColorFrameReference(char *rgbimage, RecorderInfo &recInfo)
	: rgbimage(rgbimage), recInfo(recInfo)
{
}

HRESULT STDMETHODCALLTYPE InputRecorder::ColorFrameReference::AcquireFrame(IColorFrame **colorFrame) {
	auto cf = new ColorFrame(rgbimage, recInfo);
	*colorFrame = dynamic_cast<IColorFrame*>(cf);
	return S_OK;
}


InputRecorder::DepthFrame::DepthFrame(char *depth2rgb, char *depth2xyz, RecorderInfo &recInfo)
	: depth2rgb(depth2rgb), depth2xyz(depth2xyz), recInfo(recInfo)
{
}

HRESULT InputRecorder::DepthFrame::copyDepthData(UINT capacityRgb, BYTE *rgb, UINT capacityXyz, BYTE *xyz) {
	memcpy(rgb, depth2rgb, capacityRgb);
	memcpy(xyz, depth2xyz, capacityXyz);
	return S_OK;
}

ULONG STDMETHODCALLTYPE InputRecorder::DepthFrame::Release() {
	//delete depth2rgb;
	return 0;
}

HRESULT STDMETHODCALLTYPE InputRecorder::DepthFrame::AccessUnderlyingBuffer(UINT *capacity, UINT16 **buffer) {
	return S_FALSE; // 
	*capacity = recInfo.depthSize(); // AccessUnderlyingBuffer
	*buffer = new UINT16[*capacity];
	memcpy(*buffer, (UINT16*)depth2rgb, *capacity);
	//*buffer = (UINT16*) depth2rgb;
	return S_OK;
}


InputRecorder::DepthFrameReference::DepthFrameReference(char *depth2rgb, char *depth2xyz, RecorderInfo &recInfo)
	: depth2rgb(depth2rgb), depth2xyz(depth2xyz), recInfo(recInfo)
{
}

HRESULT STDMETHODCALLTYPE InputRecorder::DepthFrameReference::AcquireFrame(IDepthFrame **colorFrame) {
	auto cf = new DepthFrame(depth2rgb, depth2xyz, recInfo);
	*colorFrame = dynamic_cast<IDepthFrame*>(cf);
	return S_OK;
}


InputRecorder::MultiSourceFrame::MultiSourceFrame(char *depth2rgb, char *depth2xyz, char *rgbimage, RecorderInfo &recInfo)
	: depth2rgb(depth2rgb), depth2xyz(depth2xyz), rgbimage(rgbimage), recInfo(recInfo)
{
}

HRESULT STDMETHODCALLTYPE InputRecorder::MultiSourceFrame::get_ColorFrameReference(_COM_Outptr_  IColorFrameReference **colorFrameReference) {
	auto cfr = new ColorFrameReference(rgbimage, recInfo);
	*colorFrameReference = dynamic_cast<IColorFrameReference*>(cfr);
	return S_OK;
}

HRESULT STDMETHODCALLTYPE InputRecorder::MultiSourceFrame::get_DepthFrameReference(_COM_Outptr_  IDepthFrameReference **depthFrameReference) {
	auto dfr = new DepthFrameReference(depth2rgb, depth2xyz, recInfo);
	*depthFrameReference = dynamic_cast<IDepthFrameReference*>(dfr);
	return S_OK;
}


InputRecorder::InputRecorder(std::string path) 
	: path(path)
{
}

InputRecorder::~InputRecorder() {
}

void InputRecorder::jumpToFrame(int frame) {
	nextFrame = frame;
}

int InputRecorder::getCurrentFrame() const {
	return currentFrame;
}

HRESULT STDMETHODCALLTYPE InputRecorder::AcquireLatestFrame(IMultiSourceFrame **multiSourceFrame) {
	//qDebug() << "AcquireLatestFrame START";
	auto t1 = std::chrono::high_resolution_clock::now();

	if (ifs.peek() == EOF) {
		currentFrame = 0;
		ifs.close();
	}
	if (!ifs.is_open()) {
		ifs.open(path, std::ifstream::in | std::ifstream::binary);
		ifs.seekg(0, std::ios::beg);
	}

	if (nextFrame != -1 && nextFrame != currentFrame) {
		ifs.close();
		ifs.open(path, std::ifstream::in | std::ifstream::binary);
		ifs.seekg(0, std::ios::beg);
		// skip x frames
		if (nextFrame != 0) {
			for (int i = 0; i < nextFrame; ++i) {
				char tmp[20]; // should be plenty
				ifs.read(tmp, 20);
				std::string stmp(tmp);
				if (stmp[0] != 'S') {
					return S_FALSE;
				}
				auto useCompression = stmp[1] == 'C';
				auto size = std::stoi(stmp.substr(2, 17));
				ifs.seekg(ifs.tellg() + std::streampos(size));
			}
		}
		currentFrame = nextFrame;
		nextFrame = -1;
	}

	boost::iostreams::filtering_stream<boost::iostreams::input> inFrame;

	char tmp[20]; // should be plenty
	ifs.read(tmp, 20);
	std::string stmp(tmp);
	if (stmp[0] != 'S') {
		return S_FALSE;
	}
	auto useCompression = stmp[1] == 'C';
	auto size = std::stoi(stmp.substr(2, 17));

	if (useCompression) {
		inFrame.push(boost::iostreams::gzip_decompressor());
		//auto compressed = new char[size];
		inFrame.push(ifs);
		//inFrame.push(boost::iostreams::array_source(compressed, size));
		//ifs.read(compressed, size);
	}
	else {
		inFrame.push(ifs);
	}

	if (depth2rgb != nullptr) {
		delete depth2rgb;
	}
	auto depth2rgbByteSize = recInfo.depth2rgbByteSize();
	depth2rgb = new char[depth2rgbByteSize];
	inFrame.read(depth2rgb, depth2rgbByteSize);

	if (depth2xyz != nullptr) {
		delete depth2xyz;
	}
	auto depth2xyzByteSize = recInfo.depth2xyzByteSize();
	depth2xyz = new char[depth2xyzByteSize];
	inFrame.read(depth2xyz, depth2xyzByteSize);

	if (rgbimage != nullptr) {
		delete rgbimage;
	}
	auto rgbimageByteSize = recInfo.rgbimageByteSize();
	rgbimage = new char[rgbimageByteSize];
	inFrame.read(rgbimage, rgbimageByteSize);

	currentFrame++;

	if (useCompression) {
		//delete compressed;
	}

	auto msf = new MultiSourceFrame(depth2rgb, depth2xyz, rgbimage, recInfo);
	*multiSourceFrame = dynamic_cast<IMultiSourceFrame*>(msf);

	auto t2 = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
	//qDebug() << "AcquireLatestFrame STOP. Took: " << duration << "ms";

	return S_OK;
}
