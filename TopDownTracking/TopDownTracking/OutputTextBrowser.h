#ifndef OUTPUT_TEXT_BROWSER_H
#define OUTPUT_TEXT_BROWSER_H

#include <QtWidgets/QTextBrowser>

class OutputTextBrowser : public QTextBrowser
{
	Q_OBJECT

private:
	std::string getTimeStr();

public:
	explicit OutputTextBrowser(QWidget *parent = 0);
	~OutputTextBrowser();

public slots:
	void logEntry(std::string line);
};

#endif // OUTPUT_TEXT_BROWSER_H