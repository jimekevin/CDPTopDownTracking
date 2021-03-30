#include <iostream>
#include <QtWidgets/QApplication>
#include <QtGui/QSurfaceFormat>
#include <librealsense2/rs.hpp>

#include "MainWindow.h"
#include "Config.h"

#ifdef APPLE
//#include "KinectManager_MacOS.h"
#include "MacOSAtrocities.hpp"
#else
#include "KinectManager_Windows.h"
#endif

#define CONFIG_DEFAULT_PATH "config/default.yaml"

int main(int argc, char **argv) try {

	QApplication app(argc, argv);
	QApplication::setOrganizationName("LS AI");
	QApplication::setOrganizationName("cdp.ai.ar.tum.de");
	QApplication::setApplicationName("Collaborative Design Platform");

#ifdef APPLE
	QSurfaceFormat format;
	format.setVersion(4, 1);
	format.setProfile(QSurfaceFormat::CoreProfile);
	format.setOption(QSurfaceFormat::DebugContext);
	QSurfaceFormat::setDefaultFormat(format);
#endif

	auto conf = Config::instance();
	conf.parseSimple(CONFIG_DEFAULT_PATH);
	auto inputMode = conf.getValueI("input");
	auto inputSource = conf.getValue("input_source");
	if (inputMode == 2) {
#ifndef APPLE
		if (KinectManager::instance().initializeFromFile(inputSource) != S_OK) {
			std::cout << "Failed loading input stream from file" << std::endl;
			return 0;
		}
#endif // APPLE
	}
	if (inputMode == 3) {
	}
	else {
#ifndef APPLE
		if (KinectManager::instance().initialize() != S_OK) {
			std::cout << "Failed loading input stream from Kinect device" << std::endl;
			return 0;
		}
#endif // APPLE
	}

	MainWindow window;
	window.show();

	app.exec();

	return 0;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
