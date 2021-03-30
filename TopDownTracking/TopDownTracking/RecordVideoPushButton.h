#pragma once

#include <QtWidgets/QPushButton>

class RecordVideoPushButton :
	public QPushButton
{
	Q_OBJECT

public:
	RecordVideoPushButton(QWidget *parent);

public slots:
	void startRecordingVideo();
	void stopRecordingVideo();

signals:
	void startedRecordingVideo();
	void stoppedRecordingVideo();
};

