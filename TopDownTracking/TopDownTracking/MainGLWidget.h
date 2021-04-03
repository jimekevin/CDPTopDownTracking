#ifndef MAIN_GL_WIDGET_H
#define MAIN_GL_WIDGET_H

#include <QtGui/QOpenGLWindow>
#include <QtGui/QOpenGLFunctions>
#include <QtGui/QOpenGLBuffer>
#include <QtGui/QOpenGLVertexArrayObject>
#include <QtGui/QOpenGLTexture>
#include <QMatrix4x4>
#include <QKeyEvent>
#include <QOpenGLDebugLogger>
#include <QtWidgets/QOpenGLWidget>
#include <chrono>
#include <fstream>

#ifdef APPLE
#include "KinectManager_MacOS.h"
#else
#include "KinectManager_Windows.h"
#endif
#include "RealsenseCameraManager.h"
#include "Map.h"
#include "Config.h"
#include "pipeline_tasks/ThresholdFilter.h"
#include "pipeline_tasks/ContourDetector.h"
//#include "pipeline_tasks/CollisionMapper.h"

class QOpenGLShaderProgram;

class MainGLWidget : public QOpenGLWidget,
	protected QOpenGLFunctions
{
	Q_OBJECT

		// OpenGL Events
public:
	explicit MainGLWidget(QWidget *parent = 0);
	~MainGLWidget();

	ThresholdFilter *thresholdFilter;
	ContourDetector *contourDetector;
	//CollisionMapper *collisionMapper;

public slots:
	void initializeGL();
	void resizeGL(int width, int height);
	void paintGL();
	void teardownGL();

	void keyPressEvent(QKeyEvent *event);
	void timerEvent(QTimerEvent *event);
	void wheelEvent(QWheelEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void showEvent(QShowEvent *event);

	void recordVideo();
	void enableVideoRecordingCompression(int state);

	void exportCamera();
	void exportSettings();
	void loadSettings();
	void resetSettings();

	void toggleMap();

	void togglePipeline();
	void togglePipelineTask(int index);

	void playPauseVideo();
	void jumpToFrame(int frame);

	void changeFpsTimer(int fpsMode);

	void setCamera(int cameraId);

	void test();

signals:
	void logMessage(std::string message);
	void setOutput(QString output);
	void startedRecordingVideo();
	void stoppedRecordingVideo();
	void hideRecordVideoButton();
	void sliderValuesChanged();
	void fpsChanged(int fps);
	void playStateChanged(bool paused);
	void updatedCurrentFrame(int frame);

private:
	// OpenGL State Information
	//QOpenGLBuffer m_vertex;
	//QOpenGLVertexArrayObject m_object;
	//QOpenGLShaderProgram *m_program;

	QOpenGLShaderProgram *cameraProgram;
	QOpenGLVertexArrayObject cameraVAO;
	QOpenGLBuffer cameraBuffer;
	QOpenGLBuffer cameraDepthBuffer;
	QOpenGLBuffer cameraTextureCoordBuffer;
	int attrLocationVertex;
	int attrLocationColor;
	int attrLocationTexCoord;
	QOpenGLTexture *cameraTexture;

	Config *conf;
	int inputMode = 0;

	RealsenseCameraManager *rcm;

	bool pipelineEnabled = true;
	std::vector<bool> enabledPipelineTasks = { true, true, true, true, true, true, true, true, true };

	cv::Mat depthMat = cv::Mat(KinectManager::HEIGHT, KinectManager::WIDTH, CV_32FC3);
	cv::Mat rgbMat   = cv::Mat(720, 1280, CV_32FC3);

	QOpenGLShaderProgram *mapProgram;
	QOpenGLVertexArrayObject mapVAO;
	QOpenGLBuffer mapBuffer;
	float mapDepth = 2.4f;

	bool videoRecording = false;
	bool videoRecordingUseCompression = false;
	int videoRecordingCount = 0;
	void captureFrame();
	std::ofstream videoRecordingOfs;
	enum PlayState { PLAYING, PAUSED, JUMP_FRAME } playState = PLAYING;
	int currentFrame = 0;

	QVector3D position = QVector3D(-0.25f, 0.0f, -2.0f);
	QVector3D direction = QVector3D(0.0f, 0.0f, 1.0f);
	QVector3D right = QVector3D(1.0f, 0.0f, 0.0f);
	QVector3D up = QVector3D(0.0f, -1.0f, 0.0f);
	float FoV = 45.0;
	float horizontalAngle = 0.0f; // 3.14f;
	float verticalAngle = 0.0f; // look at the horizon

	Map map;
	bool showMap = true;

	QOpenGLDebugLogger *logger;

	QPoint mouseStart = QPoint(0.0, 0.0);

	float speed = 0.05f; // 3 units / second
	float mouseSpeed = 0.005f;
	float deltaTime = 1.0f;

	const float CAMERA_SPEED = 0.05f;

	std::chrono::steady_clock::time_point fpsLast = std::chrono::high_resolution_clock::now();
	int fpsCount = 0;

	// Camera timer
	int cameraTimerId;

	// Private Helpers
	void updateInfo();
	void printInformation();
	void printVersionInformation();
};

#endif // MAIN_GL_WIDGET_H
