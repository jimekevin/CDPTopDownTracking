#include "OutputTextBrowser.h"

#include <iostream>
#include <chrono>
#include <ctime>
#include <sstream>

OutputTextBrowser::OutputTextBrowser(QWidget *parent)
	: QTextBrowser(parent)
{
}

OutputTextBrowser::~OutputTextBrowser()
{
}

std::string OutputTextBrowser::getTimeStr() {
	std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	std::string s(30, '\0');
	//auto format = "%Y-%m-%d %H:%M:%S";
	auto format = "%H:%M:%S";
	std::strftime(&s[0], s.size(), format, std::localtime(&now));
	return s;
}

void OutputTextBrowser::logEntry(std::string line)
{
	std::stringstream formattedLine;
	formattedLine << "[" << getTimeStr() << "]: " << line;
	append(QString::fromStdString(formattedLine.str()));
}
