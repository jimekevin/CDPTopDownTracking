#include "RecordVideoPushButton.h"

RecordVideoPushButton::RecordVideoPushButton(QWidget *parent)
	: QPushButton(parent)
{
}

void RecordVideoPushButton::startRecordingVideo() {
	setText("Stop Recording");
}

void RecordVideoPushButton::stopRecordingVideo() {
	setText("Start Recording");
}
