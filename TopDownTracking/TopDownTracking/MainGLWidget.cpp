#include "MainGLWidget.h"
#include <QtCore/QDebug>
#include <QtCore/QString>
#include <QtGui/QOpenGLShaderProgram>
#include <QtMath>
#include <QThread>
#include <QOpenGLExtraFunctions>
#include <QtGlobal>
#include <QOpenGLDebugLogger>
#include <QOpenGlTexture>
#include <ctime>
#include <sstream>
#include <chrono>
#include <QFileInfo>
#include "vertex.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cstring>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <string>
#include <iomanip>
//#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#define CONFIG_DEFAULT_PATH "config/default.yaml"
#define DATA_PATH "C:\\Projects\\CDPTopDownTracking\\data\\"
#define CONFIG_PATH "C:\\Projects\\CDPTopDownTracking\\TopDownTracking\\config\\"

#ifdef APPLE
//#include "KinectManager_MacOS.h"
#else
#include "KinectManager_Windows.h"
#endif

namespace bio = boost::iostreams;

#define CM KinectManager::instance()

// Create a colored triangle
static const Vertex sg_vertexes[] = {
  Vertex(QVector3D(0.00f,  0.75f, 1.0f), QVector3D(1.0f, 0.0f, 0.0f)),
  Vertex(QVector3D(-0.75f, -0.75f, 1.0f), QVector3D(0.0f, 1.0f, 0.0f)),
  Vertex(QVector3D(0.75f, -0.75f, 1.0f), QVector3D(0.0f, 0.0f, 1.0f))
};

static const Vertex sg_cube[] = {
	Vertex(QVector3D(-1.0f,-1.0f,-1.0f), QVector3D(0.583f,  0.771f,  0.014f)),
	Vertex(QVector3D(-1.0f,-1.0f, 1.0f), QVector3D(0.609f,  0.115f,  0.436f)),
	Vertex(QVector3D(-1.0f, 1.0f, 1.0f), QVector3D(0.327f,  0.483f,  0.844f)),
	Vertex(QVector3D(1.0f, 1.0f,-1.0f),  QVector3D(0.822f,  0.569f,  0.201f)),
	Vertex(QVector3D(-1.0f,-1.0f,-1.0f), QVector3D(0.435f,  0.602f,  0.223f)),
	Vertex(QVector3D(-1.0f, 1.0f,-1.0f), QVector3D(0.310f,  0.747f,  0.185f)),
	Vertex(QVector3D(1.0f,-1.0f, 1.0f),  QVector3D(0.597f,  0.770f,  0.761f)),
	Vertex(QVector3D(-1.0f,-1.0f,-1.0f), QVector3D(0.559f,  0.436f,  0.730f)),
	Vertex(QVector3D(1.0f,-1.0f,-1.0f),  QVector3D(0.359f,  0.583f,  0.152f)),
	Vertex(QVector3D(1.0f, 1.0f,-1.0f),  QVector3D(0.483f,  0.596f,  0.789f)),
	Vertex(QVector3D(1.0f,-1.0f,-1.0f),  QVector3D(0.559f,  0.861f,  0.639f)),
	Vertex(QVector3D(-1.0f,-1.0f,-1.0f), QVector3D(0.195f,  0.548f,  0.859f)),
	Vertex(QVector3D(-1.0f,-1.0f,-1.0f), QVector3D(0.014f,  0.184f,  0.576f)),
	Vertex(QVector3D(-1.0f, 1.0f, 1.0f), QVector3D(0.771f,  0.328f,  0.970f)),
	Vertex(QVector3D(-1.0f, 1.0f,-1.0f), QVector3D(0.406f,  0.615f,  0.116f)),
	Vertex(QVector3D(1.0f,-1.0f, 1.0f),  QVector3D(0.676f,  0.977f,  0.133f)),
	Vertex(QVector3D(-1.0f,-1.0f, 1.0f), QVector3D(0.971f,  0.572f,  0.833f)),
	Vertex(QVector3D(-1.0f,-1.0f,-1.0f), QVector3D(0.140f,  0.616f,  0.489f)),
	Vertex(QVector3D(-1.0f, 1.0f, 1.0f), QVector3D(0.997f,  0.513f,  0.064f)),
	Vertex(QVector3D(-1.0f,-1.0f, 1.0f), QVector3D(0.945f,  0.719f,  0.592f)),
	Vertex(QVector3D(1.0f,-1.0f, 1.0f),  QVector3D(0.543f,  0.021f,  0.978f)),
	Vertex(QVector3D(1.0f, 1.0f, 1.0f),  QVector3D(0.279f,  0.317f,  0.505f)),
	Vertex(QVector3D(1.0f,-1.0f,-1.0f),  QVector3D(0.167f,  0.620f,  0.077f)),
	Vertex(QVector3D(1.0f, 1.0f,-1.0f),  QVector3D(0.347f,  0.857f,  0.137f)),
	Vertex(QVector3D(1.0f,-1.0f,-1.0f),  QVector3D(0.055f,  0.953f,  0.042f)),
	Vertex(QVector3D(1.0f, 1.0f, 1.0f),  QVector3D(0.714f,  0.505f,  0.345f)),
	Vertex(QVector3D(1.0f,-1.0f, 1.0f),  QVector3D(0.783f,  0.290f,  0.734f)),
	Vertex(QVector3D(1.0f, 1.0f, 1.0f),  QVector3D(0.722f,  0.645f,  0.174f)),
	Vertex(QVector3D(1.0f, 1.0f,-1.0f),  QVector3D(0.302f,  0.455f,  0.848f)),
	Vertex(QVector3D(-1.0f, 1.0f,-1.0f), QVector3D(0.225f,  0.587f,  0.040f)),
	Vertex(QVector3D(1.0f, 1.0f, 1.0f),  QVector3D(0.517f,  0.713f,  0.338f)),
	Vertex(QVector3D(-1.0f, 1.0f,-1.0f), QVector3D(0.053f,  0.959f,  0.120f)),
	Vertex(QVector3D(-1.0f, 1.0f, 1.0f), QVector3D(0.393f,  0.621f,  0.362f)),
	Vertex(QVector3D(1.0f, 1.0f, 1.0f),  QVector3D(0.673f,  0.211f,  0.457f)),
	Vertex(QVector3D(-1.0f, 1.0f, 1.0f), QVector3D(0.820f,  0.883f,  0.371f)),
	Vertex(QVector3D(1.0f,-1.0f, 1.0f),  QVector3D(0.982f,  0.099f,  0.879f))
};

static const GLfloat sg_cube_vertices[] = {
	-1.0f,-1.0f,-1.0f, // triangle 1 : begin
	-1.0f,-1.0f, 1.0f,
	-1.0f, 1.0f, 1.0f, // triangle 1 : end
	1.0f, 1.0f,-1.0f, // triangle 2 : begin
	-1.0f,-1.0f,-1.0f,
	-1.0f, 1.0f,-1.0f, // triangle 2 : end
	1.0f,-1.0f, 1.0f,
	-1.0f,-1.0f,-1.0f,
	1.0f,-1.0f,-1.0f,
	1.0f, 1.0f,-1.0f,
	1.0f,-1.0f,-1.0f,
	-1.0f,-1.0f,-1.0f,
	-1.0f,-1.0f,-1.0f,
	-1.0f, 1.0f, 1.0f,
	-1.0f, 1.0f,-1.0f,
	1.0f,-1.0f, 1.0f,
	-1.0f,-1.0f, 1.0f,
	-1.0f,-1.0f,-1.0f,
	-1.0f, 1.0f, 1.0f,
	-1.0f,-1.0f, 1.0f,
	1.0f,-1.0f, 1.0f,
	1.0f, 1.0f, 1.0f,
	1.0f,-1.0f,-1.0f,
	1.0f, 1.0f,-1.0f,
	1.0f,-1.0f,-1.0f,
	1.0f, 1.0f, 1.0f,
	1.0f,-1.0f, 1.0f,
	1.0f, 1.0f, 1.0f,
	1.0f, 1.0f,-1.0f,
	-1.0f, 1.0f,-1.0f,
	1.0f, 1.0f, 1.0f,
	-1.0f, 1.0f,-1.0f,
	-1.0f, 1.0f, 1.0f,
	1.0f, 1.0f, 1.0f,
	-1.0f, 1.0f, 1.0f,
	1.0f,-1.0f, 1.0f
};
static const GLfloat sg_cube_color[] = {
	0.583f,  0.771f,  0.014f,
	0.609f,  0.115f,  0.436f,
	0.327f,  0.483f,  0.844f,
	0.822f,  0.569f,  0.201f,
	0.435f,  0.602f,  0.223f,
	0.310f,  0.747f,  0.185f,
	0.597f,  0.770f,  0.761f,
	0.559f,  0.436f,  0.730f,
	0.359f,  0.583f,  0.152f,
	0.483f,  0.596f,  0.789f,
	0.559f,  0.861f,  0.639f,
	0.195f,  0.548f,  0.859f,
	0.014f,  0.184f,  0.576f,
	0.771f,  0.328f,  0.970f,
	0.406f,  0.615f,  0.116f,
	0.676f,  0.977f,  0.133f,
	0.971f,  0.572f,  0.833f,
	0.140f,  0.616f,  0.489f,
	0.997f,  0.513f,  0.064f,
	0.945f,  0.719f,  0.592f,
	0.543f,  0.021f,  0.978f,
	0.279f,  0.317f,  0.505f,
	0.167f,  0.620f,  0.077f,
	0.347f,  0.857f,  0.137f,
	0.055f,  0.953f,  0.042f,
	0.714f,  0.505f,  0.345f,
	0.783f,  0.290f,  0.734f,
	0.722f,  0.645f,  0.174f,
	0.302f,  0.455f,  0.848f,
	0.225f,  0.587f,  0.040f,
	0.517f,  0.713f,  0.338f,
	0.053f,  0.959f,  0.120f,
	0.393f,  0.621f,  0.362f,
	0.673f,  0.211f,  0.457f,
	0.820f,  0.883f,  0.371f,
	0.982f,  0.099f,  0.879f
};

/*
static const GLfloat sg_cube_vertices[] = {
	1.0f, 1.0f, 0.0f,
	-1.0f, 1.0f, 0.0f,
	-1.0f, -1.0f, 0.0f,
	1.0f, 1.0f, 0.0f,
	1.0f, -1.0f, 0.0f,
	-1.0f, -1.0f, 0.0f,
	1.0f, 1.0f, 0.0f,
};*/

// e.g. Valid point: {X=-1.55726588 Y=1.33272767 Z=2.19400001 }
static const GLfloat map_plane_vertices[] = {
	-2.0f, -2.0f, 0.5f,
	2.0f, -2.0f, 0.5f,
	2.0f, 2.0f, 0.5f,
	-2.0f, -2.0f, 0.5f,
	-2.0f, 2.0f, 0.5f,
	2.0f, 2.0f, 0.5f,
};

#define MAKE_RECT(x, y, z, w, h, r, g, b) { \
	{ x - w/2, y - h/2, z, r, g, b }, \
	{ x + w/2, y - h/2, z, r, g, b }, \
	{ x + w/2, y + h/2, z, r, g, b }, \
	{ x - w/2, y - h/2, z, r, g, b }, \
	{ x - w/2, y + h/2, z, r, g, b }, \
	{ x + w/2, y + h/2, z, r, g, b }, \
}
float planeWidth = 1.2f;
float planeHeight = 0.6f;
float planeX = -0.25; 
static const GLfloat map_vertices[][6][6] = {
	//MAKE_RECT(0.0f, 0.0f, 0.50f, 4.0f, 4.0f, 0.5f, 0.5f, 0.5f), // Plane
	//MAKE_RECT(0.0f, 0.0f, 0.4999f, 1.0f, 1.0f, 0.0f, 0.7f, 0.0f), // Building 1
	//MAKE_RECT(1.0f, 1.0f, 0.4999f, 0.5f, 0.5f, 0.0f, 1.0f, 0.2f), // Building 2
	MAKE_RECT(-0.2f, 0.25f, 0.0f, 1.2f, 0.65f, 0.5f, 0.5f, 0.5f), // Plane
	MAKE_RECT(0.0f, 0.0f, +0.0001f, 1.0f, 1.0f, 0.0f, 0.7f, 0.0f), // Building 1
};
/*static const GLfloat map_vertices[][6][6] = { {
-0.5f, 0.5f, 0.5f, 1.0f, 0.0f, 0.0f, // Top-left
0.5f, 0.5f, 0.5f, 0.0f, 1.0f, 0.0f, // Top-right
0.5f, -0.5f, 0.5f, 0.0f, 0.0f, 1.0f, // Bottom-right

0.5f, -0.5f, 0.5f, 0.0f, 0.0f, 1.0f, // Bottom-right
-0.5f, -0.5f, 0.5f, 1.0f, 1.0f, 1.0f, // Bottom-left
-0.5f, 0.5f, 0.5f, 1.0f, 0.0f, 0.0f  // Top-left
} };*/
/*static const std::vector<std::vector<float[6]>>map_vertices{
	MAKE_RECT(0.0f, 0.0f, 4.0f, 4.0f, 0.5f, 0.5f, 0.5f), // Plane
	MAKE_RECT(0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.7f, 0.0f), // Building 1
};*/

MainGLWidget::MainGLWidget(QWidget *parent)
	: QOpenGLWidget(parent)
{
	conf = &Config::instance();
	conf->parseSimple(CONFIG_DEFAULT_PATH);
	inputMode = conf->getValueI("input");

	rcm = new RealsenseCameraManager();
	rcm->init();
	//rs2::threshold_filter thresholdFilter{};
	//rcm->addFilter(thresholdFilter);

	//thresholdFilter = new ThresholdFilter(-10.0f, 10.0f, -10.0f, 10.0f, 0.5f, 3.0f);
	//contourDetector = new ContourDetector(0.8f, 1.0f);
	// K1NECT rework CollisionMapper
	//collisionMapper = new CollisionMapper(contourDetector, &map, CM.color2camera);

	// Create pipeline
	//pipeline.addProcessingTask(thresholdFilter);
	//pipeline.addProcessingTask(contourDetector);
	// K1NECT
	//pipeline.addProcessingTask(collisionMapper);

	// Make window activ to recieve key strokes 
	// and be able to handle them in here
	setFocus();

	updateInfo();
}

// K1NECT disable emit hideRecordVideoButton
void MainGLWidget::showEvent(QShowEvent *event) {
	// K1NECT
	//if (CM.loadedKinect != CM.KINECT) {
	//	emit hideRecordVideoButton();
	//}

	loadSettings();
}

MainGLWidget::~MainGLWidget()
{
	if (inputMode == 3) {
		delete rcm;
	}
}

void MainGLWidget::initializeGL()
{
	// Initialize OpenGL Backend
	initializeOpenGLFunctions();
	//connect(context(), SIGNAL(aboutToBeDestroyed()), this, SLOT(teardownGL()), Qt::DirectConnection);
	printVersionInformation();

	// Set global information
	//glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
	//glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClearColor(0.9f, 0.9f, 0.99f, 1.0f);
	glClearDepth(1.0f);
	glEnable(GL_PROGRAM_POINT_SIZE);
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	//glEnable(GL_DEBUG_OUTPUT);
	QSurfaceFormat format;
	format.setMajorVersion(4);
	format.setMinorVersion(1);
	format.setProfile(QSurfaceFormat::CoreProfile);
	format.setOption(QSurfaceFormat::DebugContext);

	//QOpenGLContext *ctx = QOpenGLContext::currentContext();
	auto ctx = (QOpenGLContext*)context();
	ctx->setFormat(format);
	logger = new QOpenGLDebugLogger(this);
	logger->initialize(); // initializes in the current context, i.e. ctx

	// Cull triangles which normal is not towards the camera = you cannnot go inside objects
	//glEnable(GL_CULL_FACE);

	{
		//QFileInfo info("./shaders/point_cloud.vert");
		//qDebug() << info.absoluteFilePath();
		cameraProgram = new QOpenGLShaderProgram();
		cameraProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, "./shaders/point_cloud.vert");
		cameraProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, "./shaders/point_cloud.frag");
		cameraProgram->link();

		attrLocationVertex = cameraProgram->attributeLocation("vertex");
		attrLocationTexCoord = cameraProgram->attributeLocation("texCoord");
		cameraTexture = new QOpenGLTexture(QOpenGLTexture::Target2D);
		cameraTexture->create();
		cameraTexture->setSize(848, 480, 1);
		cameraTexture->allocateStorage();
		cameraTexture->setMagnificationFilter(QOpenGLTexture::Filter::Linear);
		cameraTexture->setMinificationFilter(QOpenGLTexture::Filter::Linear);
		cameraTexture->setWrapMode(QOpenGLTexture::WrapMode::ClampToEdge);
		//glGenTextures(1, &cameraTexture);

		cameraProgram->bind();
		cameraProgram->setAttributeValue("color", QVector3D(1.0, 0.0, 0.0));
		cameraProgram->release();
	}

	{
		mapProgram = new QOpenGLShaderProgram();
		mapProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, "./shaders/map_view.vert");
		mapProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, "./shaders/map_view.frag");
		mapProgram->link();
		mapProgram->release();
	}

	cameraTimerId = startTimer(16);

	updateInfo();
}

void MainGLWidget::resizeGL(int width, int height)
{
	// Currently we are not handling width/height changes.
	(void)width;
	(void)height;
}

void MainGLWidget::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (!cameraDepthBuffer.isCreated()) {
		return;
	}

	QMatrix4x4 view;
	view.lookAt(position, position + direction, QVector3D(0, -1, 0));

	QMatrix4x4 projection;
	projection.perspective(FoV, width() / (GLfloat)height(), 0.1, 100.0);

	if (showMap) {
		mapProgram->bind();
		{
			QMatrix4x4 model;
			model.translate(0.0f, 0.0f, mapDepth);
			mapProgram->setUniformValue("model", model);

			mapProgram->setUniformValue("view", view);
			mapProgram->setUniformValue("projection", projection);

			auto attrLocationVertex = mapProgram->attributeLocation("vertex");
			auto attrLocationColor = mapProgram->attributeLocation("color");

			mapVAO.bind();

			mapBuffer.bind();
			glEnableVertexAttribArray(attrLocationVertex);
			glVertexAttribPointer(attrLocationVertex, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), 0);
			glEnableVertexAttribArray(attrLocationColor);
			glVertexAttribPointer(attrLocationColor, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
			for (int i = 0; i < sizeof(map_vertices) / 6 / 6; i++) {
				glDrawArrays(GL_TRIANGLES, i * 6, 6);
			}
			glDisableVertexAttribArray(attrLocationVertex);
			glDisableVertexAttribArray(attrLocationColor);

			mapBuffer.release();

			mapVAO.release();
		}
		mapProgram->release();
	}

	cameraProgram->bind();
	{
		cameraProgram->setUniformValue("view", view);
		cameraProgram->setUniformValue("projection", projection);

		cameraVAO.bind();

		glEnableVertexAttribArray(attrLocationVertex);
		cameraDepthBuffer.bind();
		glVertexAttribPointer(attrLocationVertex, 3, GL_FLOAT, GL_FALSE, 0, 0);
		cameraDepthBuffer.release();

		glEnableVertexAttribArray(attrLocationTexCoord);
		cameraTextureCoordBuffer.bind();
		glVertexAttribPointer(attrLocationTexCoord, 2, GL_FLOAT, GL_FALSE, 0, 0);
		//cameraTextureCoordBuffer.release();

		cameraTexture->bind();

		glDrawArrays(GL_POINTS, 0, cameraDepthBuffer.size());

		glDisableVertexAttribArray(attrLocationVertex);
		glDisableVertexAttribArray(attrLocationTexCoord);

		cameraDepthBuffer.release();
		cameraTextureCoordBuffer.release();
		cameraTexture->release();

		cameraVAO.release();
	}
	cameraProgram->release();

	fpsCount++;
	auto fpsNow = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(fpsNow - fpsLast).count();
	if (duration >= 1000) {
		emit fpsChanged(fpsCount);
		fpsCount = 0;
		fpsLast = std::chrono::high_resolution_clock::now();
	}
}

void MainGLWidget::timerEvent(QTimerEvent *)
{
	long frameResult;
	// K1NECT
	if (playState == PlayState::PLAYING || playState == PlayState::JUMP_FRAME) {
		if (playState == PlayState::JUMP_FRAME) {
			playState = PlayState::PAUSED;
		}
	}

	// Enable or disable certain filters
	if (pipelineEnabled) {
		for (int i = 0; i < enabledPipelineTasks.size(); ++i) {
			rcm->enableFilter(i, enabledPipelineTasks[i]);
		}
	}
	// Poll latest frames
	if (!rcm->pollFrames()) {
		return;
	}
	auto rs = rcm->processFrames();

	// Update frame id
	emit updatedCurrentFrame(rs.points.get_frame_number());

	// K1NECT
	//if (videoRecording) {
	//	captureFrame();
	//}

	// Write camera frame buffers
	{
		cameraProgram->bind();

		if (cameraVAO.isCreated()) {
			cameraVAO.destroy();
		}
		cameraVAO.create();
		cameraVAO.bind();

		// depth data
		cameraDepthBuffer.create();
		cameraDepthBuffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
		cameraDepthBuffer.bind();
		auto depthSize = rs.points.size() * sizeof(rs2::vertex);
		cameraDepthBuffer.allocate(depthSize);
		auto depthBufferDest = cameraDepthBuffer.mapRange(0, depthSize, QOpenGLBuffer::RangeInvalidateBuffer | QOpenGLBuffer::RangeWrite);
		memcpy(depthBufferDest, rs.points.get_vertices(), depthSize);
		cameraDepthBuffer.unmap();
		cameraDepthBuffer.release();

		// texture coordinates
		cameraTextureCoordBuffer.create();
		cameraTextureCoordBuffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
		cameraTextureCoordBuffer.bind();
		auto textureCoordSize = rs.points.size() * sizeof(rs2::texture_coordinate);
		cameraTextureCoordBuffer.allocate(textureCoordSize);
		auto rgbBufferDest = cameraTextureCoordBuffer.mapRange(0, textureCoordSize, QOpenGLBuffer::RangeInvalidateBuffer | QOpenGLBuffer::RangeWrite);
		memcpy(rgbBufferDest, rs.points.get_texture_coordinates(), textureCoordSize);
		cameraTextureCoordBuffer.unmap();
		cameraTextureCoordBuffer.release();

		// rgb data
		//cameraRGBBuffer.create();
		//cameraRGBBuffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
		//cameraRGBBuffer.bind();
		//auto rgbSize = rs.colorFrame.get_data_size();
		//cameraRGBBuffer.allocate(rgbSize);
		//auto rgbBufferDest = cameraRGBBuffer.mapRange(0, rgbSize, QOpenGLBuffer::RangeInvalidateBuffer | QOpenGLBuffer::RangeWrite);
		//memcpy(rgbBufferDest, rs.colorFrame.get_data(), rgbSize);
		////CM.writeRgbData(rgbBufferDest);
		//cameraRGBBuffer.unmap();
		//cameraRGBBuffer.release();
		// texture data
		cameraTexture->bind();
		//glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, rs.colorFrame.get_width(), rs.colorFrame.get_height(), 0, GL_RGB, GL_UNSIGNED_BYTE, rs.colorFrame.get_data());
		//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, rgbMat.cols, rgbMat.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, rgbMat.data);
		//cameraTexture->setData(QOpenGLTexture::PixelFormat::RGB, QOpenGLTexture::PixelType::UInt8, rs.colorFrame.get_data());
		cameraTexture->release();

		cameraVAO.release();
		cameraProgram->release();
	}

	// Write map if something changed
	{
		if (mapVAO.isCreated()) {
			mapVAO.destroy();
		}
		mapVAO.create();
		mapVAO.bind();

		mapBuffer.create();
		mapBuffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
		mapBuffer.bind();
		mapBuffer.allocate(map.data(), map.size());
		mapBuffer.release();

		mapVAO.release();
		mapProgram->release();
	}

	GLenum glErr;
	while ((glErr = glGetError()) != GL_NO_ERROR)
	{
		qDebug() << glErr;
		const QList<QOpenGLDebugMessage> messages = logger->loggedMessages();
		for (const QOpenGLDebugMessage &message : messages)
			qDebug() << message;
	}

	update();
}

// K1NECT
void MainGLWidget::captureFrame() {
	return;

#ifndef APPLE
	if (videoRecordingCount >= CM.frameCount) {
		return;
	}

	auto rawSize = (
		(CM.WIDTH * CM.HEIGHT * sizeof(ColorSpacePoint)) +
		(CM.WIDTH * CM.HEIGHT * sizeof(CameraSpacePoint)) +
		(CM.COLORWIDTH * CM.COLORHEIGHT * 4 * sizeof(unsigned char))
	);
	std::stringstream headerInformation;
	if (videoRecordingUseCompression) {
		/*boost::iostreams::filtering_stream<boost::iostreams::input> in; // (compressed);
		in.push(boost::iostreams::gzip_compressor());
		in.push(boost::iostreams::array_source(reinterpret_cast<const char*>(CM.depth2rgb), rawSize));
		std::stringstream compressed(std::stringstream::in | std::stringstream::out | std::stringstream::binary);
		auto size = boost::iostreams::copy(in, compressed);*/

		using RawBuffer = std::vector<char>;
		using Device = bio::back_insert_device<RawBuffer>;

		RawBuffer compressed_buffer; // optionally reserve some size
		//compressed_buffer.clear(); // do not shrink_to_fit optimizing allocation
		{
			bio::filtering_ostreambuf filter;
			filter.push(bio::gzip_compressor());
			filter.push(Device{ compressed_buffer });

			std::copy_n(reinterpret_cast<char const*>(CM.depth2rgb),
				//sizeof(frame) - offsetof(Frame, buf1),
				rawSize,
				std::ostreambuf_iterator<char>(&filter)
			);
		}

		headerInformation << "SC" << std::setw(17) << compressed_buffer.size() << "E";
		videoRecordingOfs << headerInformation.str();
		videoRecordingOfs.write(compressed_buffer.data(), compressed_buffer.size());
	}
	else {
		headerInformation << "S " << std::setw(17) << rawSize << "E";
		videoRecordingOfs << headerInformation.str();
		videoRecordingOfs.write(reinterpret_cast<const char*>(CM.depth2rgb), rawSize);
	}
	videoRecordingCount = CM.frameCount;
#endif // APPLE
}

// K1NECT disabled Z,U,I key press (screenshots)
void MainGLWidget::keyPressEvent(QKeyEvent *event) {
	switch (event->key()) {
	case Qt::Key::Key_Up:
	case Qt::Key::Key_W:
		//view.translate(0, CAMERA_SPEED, 0);
		position += up * deltaTime * speed;
		break;
	case Qt::Key::Key_Down:
	case Qt::Key::Key_S:
		//view.translate(0, -CAMERA_SPEED, 0);
		position -= up * deltaTime * speed;
		break;
	case Qt::Key::Key_Right:
	case Qt::Key::Key_D:
		//view.translate(CAMERA_SPEED, 0, 0);
		position += right * deltaTime * speed;
		break;
	case Qt::Key::Key_Left:
	case Qt::Key::Key_A:
		//view.translate(-CAMERA_SPEED, 0, 0);
		position -= right * deltaTime * speed;
		break;
	case Qt::Key::Key_Q:
		//view.translate(CAMERA_SPEED, 0, 0);
		position += direction * deltaTime * speed;
		break;
	case Qt::Key::Key_E:
		//view.translate(-CAMERA_SPEED, 0, 0);
		position -= direction * deltaTime * speed;
		break;

	case Qt::Key::Key_Z:
	case Qt::Key::Key_U:
	case Qt::Key::Key_I:
	{
		// K1NECT
		return;
		std::time_t result = std::time(nullptr);
		std::stringstream path;
		//path << "C:\\Users\\Kevin Bein\\Downloads\\" << result << ".png";
		path << DATA_PATH << result << ".png";
		if (event->key() == Qt::Key::Key_Z) {
			CM.saveRGBImage(path.str());
		}
		else {
			cv::Mat image = rgbMat.clone();
			cv::cvtColor(image, image, cv::COLOR_RGB2BGRA);
			cv::flip(image, image, 1);
			//image.convertTo(image, CV_8UC3);
			//image.convertTo(image, CV_8UC4);
			//CM.saveRGBImage(path.str(), rgbMat.data, rgbMat.cols, image.rows);
			//cv::imwrite(path.str(), image);
			if (event->key() == Qt::Key::Key_U) {
				cv::imshow(path.str(), image);
			}
			else { // I
				cv::imwrite(path.str(), image);
			}
		}

		std::stringstream message;
		message << "Saved RGB snapshot at " << path.str() << "";
		qDebug() << message.str().c_str();

		std::stringstream title;
		title << "CDP Tests - " << message.str();
		window()->setWindowTitle(title.str().c_str());
	}
	break;

	case Qt::Key::Key_1:
		mapDepth -= 0.2f;
		break;

	case Qt::Key::Key_2:
		mapDepth += 0.2f;
		break;

	case Qt::Key::Key_3:
		map.markBuilding(2);
		break;

	case Qt::Key::Key_4:
		map.unmarkBuilding(2);
		break;
	}
	//qDebug() << "press key: " << event->key() << " (" << event->text() << ") ";
	event->accept();
	updateInfo();
}

void MainGLWidget::mousePressEvent(QMouseEvent *event) {
	mouseStart = event->pos();
}

void MainGLWidget::mouseReleaseEvent(QMouseEvent *) {
	setFocus();
}

void MainGLWidget::mouseMoveEvent(QMouseEvent *event) {
	if (event->buttons() == Qt::LeftButton) {
		const auto deltaTime = 1.0;
		auto mouseX = event->pos().x() - mouseStart.x();
		auto mouseY = event->pos().y() - mouseStart.y();
		//qDebug() << "Mouse move: " << mouseX << ", " << mouseY;
		horizontalAngle += mouseSpeed * deltaTime * mouseX;
		verticalAngle += mouseSpeed * deltaTime * mouseY;
		direction = QVector3D(
			cos(verticalAngle) * sin(horizontalAngle),
			sin(verticalAngle),
			cos(verticalAngle) * cos(horizontalAngle)
		);
		right = QVector3D(
			-sin(horizontalAngle - 3.14f / 2.0f),
			0,
			-cos(horizontalAngle - 3.14f / 2.0f)
		);
		up = QVector3D::crossProduct(right, direction);

		mouseStart = event->pos();

		updateInfo();
	}
	event->accept();
}

void MainGLWidget::wheelEvent(QWheelEvent *event) {
	auto numDegrees = event->angleDelta() / 8;
	auto numSteps = numDegrees / 15;

	if (event->angleDelta().x() == 0) {
		FoV -= 5.0f * numSteps.y();

		updateInfo();
	}
	event->accept();
}

void MainGLWidget::teardownGL()
{
	killTimer(cameraTimerId);

	cameraDepthBuffer.destroy();
	cameraTextureCoordBuffer.destroy();
	cameraTexture->destroy();
	cameraVAO.destroy();

	delete cameraProgram;
}

void MainGLWidget::updateInfo()
{
	printInformation();
	QWidget::update();
}

void MainGLWidget::printInformation()
{
	std::stringstream output;

	output << "QVector3D position    = QVector3D(" << position.x() << ", " << position.y() << ", " << position.z() << ");" << std::endl;
	output << "QVector3D direction   = QVector3D(" << direction.x() << ", " << direction.y() << ", " << direction.z() << ");" << std::endl;
	output << "QVector3D right       = QVector3D(" << right.x() << ", " << right.y() << ", " << right.z() << ");" << std::endl;
	output << "QVector3D up          = QVector3D(" << up.x() << ", " << up.y() << ", " << up.z() << ");" << std::endl;
	output << "float FoV             = " << FoV << ";" << std::endl;
	output << "float horizontalAngle = " << horizontalAngle << ";" << std::endl;
	output << "float verticalAngle   = " << verticalAngle << ";" << std::endl;

	output << "" << std::endl;

	emit setOutput(QString::fromStdString(output.str()));
}

void MainGLWidget::exportCamera() {
	std::cout << std::fixed << std::showpoint << std::setprecision(1)
			  << "\n"
			  << "QVector3D position    = QVector3D(" << position.x() << ", " << position.y() << ", " << position.z() << ");\n" 
			  << "QVector3D direction   = QVector3D(" << direction.x() << ", " << direction.y() << ", " << direction.z() << ");\n"
			  << "QVector3D right       = QVector3D(" << right.x() << ", " << right.y() << ", " << right.z() << ");\n"
			  << "QVector3D up          = QVector3D(" << up.x() << ", " << up.y() << ", " << up.z() << ");\n" 
			  << "float FoV             = " << FoV << ";\n"
			  << "float horizontalAngle = " << horizontalAngle << ";\n"
			  << "float verticalAngle   = " << verticalAngle << ";\n"
			  << "\n";
}

void MainGLWidget::printVersionInformation()
{
	QString glType;
	QString glVersion;
	QString glProfile;

	// Get Version Information
	auto ctx = (QOpenGLContext*) this->context();
	glType = (ctx->isOpenGLES()) ? "OpenGL ES" : "OpenGL";
	glVersion = reinterpret_cast<const char*>(glGetString(GL_VERSION));

	// Get Profile Information
#define CASE(c) case QSurfaceFormat::c: glProfile = #c; break
	switch (format().profile())
	{
		CASE(NoProfile);
		CASE(CoreProfile);
		CASE(CompatibilityProfile);
	}
#undef CASE

	qDebug() << "Qt Version: " << QT_VERSION_STR;

	// qPrintable() will print our QString w/o quotes around it.
	qDebug() << qPrintable(glType) << qPrintable(glVersion) << "(" << qPrintable(glProfile) << ")";
}

void MainGLWidget::test() {
}

void MainGLWidget::recordVideo() {
	if (!videoRecording) {
		std::time_t result = std::time(nullptr);
		std::stringstream path;
		path << "C:\\Projects\\cdp_tests\\data\\" << result << (videoRecordingUseCompression ? "_c" : "_u");
		videoRecordingOfs.open(path.str(), std::ofstream::binary | std::ofstream::app);

		videoRecording = true;

		emit startedRecordingVideo();
		qDebug() << "Stopped video recording";
	}
	else {
		videoRecording = false;

		videoRecordingOfs.close();

		emit stoppedRecordingVideo();
		qDebug() << "Stopped video recording ";
	}
}

void MainGLWidget::enableVideoRecordingCompression(int state) {
	videoRecordingUseCompression = state == Qt::CheckState::Checked;
}

void MainGLWidget::exportSettings() {
	//std::cout << std::fixed << std::showpoint << std::setprecision(1)
	//	      << "\n"
	//          << "task_thresholds_minX: " << thresholdFilter->minX << "f\n"
	//          << "task_thresholds_maxX: " << thresholdFilter->maxX << "f\n"
	//          << "task_thresholds_minY: " << thresholdFilter->minY << "f\n"
	//          << "task_thresholds_maxY: " << thresholdFilter->maxY << "f\n"
	//          << "task_thresholds_minZ: " << thresholdFilter->minZ << "f\n"
	//          << "task_thresholds_maxZ: " << thresholdFilter->maxZ << "f\n"
	//          << "task_countour_detector_threshold1: " << contourDetector->threshold1 << "f\n"
	//          << "task_countour_detector_threshold2: " << contourDetector->threshold2 << "f\n"
	//          << "\n";
}

void MainGLWidget::loadSettings() {
	//auto taskThresholdsMinX = conf->getValueF("task_thresholds_minX");
	//auto taskThresholdsMaxX = conf->getValueF("task_thresholds_maxX");
	//auto taskThresholdsMinY = conf->getValueF("task_thresholds_minY");
	//auto taskThresholdsMaxY = conf->getValueF("task_thresholds_maxY");
	//auto taskThresholdsMinZ = conf->getValueF("task_thresholds_minZ");
	//auto taskThresholdsMaxZ = conf->getValueF("task_thresholds_maxZ");
	//auto taskCountourDetectorThreshold1 = conf->getValueF("task_countour_detector_threshold1");
	//auto taskCountourDetectorThreshold2 = conf->getValueF("task_countour_detector_threshold2");

	//thresholdFilter->minX = taskThresholdsMinX != INFINITY ? taskThresholdsMinX : thresholdFilter->minX;
	//thresholdFilter->maxX = taskThresholdsMaxX != INFINITY ? taskThresholdsMaxX : thresholdFilter->maxX;
	//thresholdFilter->minY = taskThresholdsMinY != INFINITY ? taskThresholdsMinY : thresholdFilter->minY;
	//thresholdFilter->maxY = taskThresholdsMaxY != INFINITY ? taskThresholdsMaxY : thresholdFilter->maxY;
	//thresholdFilter->minZ = taskThresholdsMinZ != INFINITY ? taskThresholdsMinZ : thresholdFilter->minZ;
	//thresholdFilter->maxZ = taskThresholdsMaxZ != INFINITY ? taskThresholdsMaxZ : thresholdFilter->maxZ;
	//contourDetector->threshold1 = taskCountourDetectorThreshold1 != INFINITY ? taskCountourDetectorThreshold1 : contourDetector->threshold1;
	//contourDetector->threshold2 = taskCountourDetectorThreshold2 != INFINITY ? taskCountourDetectorThreshold2 : contourDetector->threshold2;

	//emit sliderValuesChanged();
}

void MainGLWidget::resetSettings() {
	//thresholdFilter->minX = -10.0f;
	//thresholdFilter->maxX = 10.0f;
	//thresholdFilter->minY = -10.0f;
	//thresholdFilter->maxY = 10.0f;
	//thresholdFilter->minZ = -10.0f;
	//thresholdFilter->maxZ = 10.0f;
	//contourDetector->threshold1 = 0.8f;
	//contourDetector->threshold2 = 1.0f;

	//emit sliderValuesChanged();
}

void MainGLWidget::toggleMap() {
	showMap = !showMap;
}

void MainGLWidget::togglePipeline() {
	pipelineEnabled = !pipelineEnabled;
}

void MainGLWidget::togglePipelineTask(int index) {
	if (index < 0 || index >= enabledPipelineTasks.size()) {
		return;
	}
	enabledPipelineTasks[index] = !enabledPipelineTasks[index];
}

void MainGLWidget::playPauseVideo() {
	playState = playState == PlayState::PAUSED ? PlayState::PLAYING : PlayState::PAUSED;
}

void MainGLWidget::jumpToFrame(int frame) {
	// K1NECT
	return;
#ifndef APPLE
	CM.jumpToFrame(frame);
	playState = PlayState::JUMP_FRAME;
#endif // APPLE
}

void MainGLWidget::changeFpsTimer(int fpsMode) {
	killTimer(cameraTimerId);
	cameraTimerId = startTimer(fpsMode == 1 ? std::chrono::milliseconds(16) : std::chrono::milliseconds(8));
}

template <typename T>
std::vector<T> json_array_as_vector(boost::property_tree::ptree const& pt, boost::property_tree::ptree::key_type const& key)
{
	std::vector<T> r;
	for (auto& item : pt.get_child(key))
		r.push_back(item.second.get_value<T>());
	return r;
}

void MainGLWidget::setCamera(int cameraId) {

	switch (cameraId) {
	case 0:
		position = QVector3D(-0.25f, 0.0f, -2.0f);
		direction = QVector3D(0.0f, 0.0f, 1.0f);
		right = QVector3D(1.0f, 0.0f, 0.0f);
		up = QVector3D(0.0f, -1.0f, 0.0f);
		FoV = 45.0;
		horizontalAngle = 0.0f;
		verticalAngle = 0.0f;
		break;
	case 1:
		position = QVector3D(-2.95611, 0, -0.54167);
		direction = QVector3D(0.803955, 0.0399893, 0.593344);
		right = QVector3D(-0.593178, 0, 0.805071);
		up = QVector3D(-0.0321943, -0.9992, -0.0237208);
		FoV = 45;
		horizontalAngle = 0.934999;
		verticalAngle = 0.04;
		break;
	case 2:
		position = QVector3D(-3.73998, 0, 2.37933);
		direction = QVector3D(0.977134, -0.00499996, -0.212565);
		right = QVector3D(0.213346, 0, 0.976977);
		up = QVector3D(0.00488485, -0.999987, -0.00106672);
		FoV = 45;
		horizontalAngle = 1.785;
		verticalAngle = -0.00499999;
		break;
	}

	/*
	TODO: try later again
	
	namespace pt = boost::property_tree;
	try {
		std::stringstream path;
		path << CONFIG_PATH << "camera.json";

		pt::ptree root;
		pt::read_json(path.str(), root);

		std::vector<std::string> fruits;
		for (pt::ptree::value_type &camera : root.get_child("cameras")) {
			auto id = camera.second.get_child("id");
			auto positionArr = camera.second.get_child("position");
			auto position = json_array_as_vector<float>(positionArr, "position");

			//"id": 2,
			//"position" : [-0.25, 0.0, -2.0],
			//"direction" : [0.0, 0.0, 1.0],
			//"right" : [-1.0, 0.0, 0.0],
			//"up" : [0.0, 1.0, 0.0],
			//"FoV" : 45.0,
			//"horizontalAngle" : 0.0,
			//"verticalAngle" : 0.0
		}
	}
	catch (std::exception const& e)
	{
		std::cerr << e.what() << std::endl;
	}*/
}
