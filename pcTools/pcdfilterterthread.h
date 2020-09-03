#ifndef PCDFILTERTHREAD_H
#define PCDFILTERTHREAD_H

#include <QThread>
#include "helpers/dirprocess.h"
#include "saveData/pointcloudwriter.h"

class PCDFilterThread : public QThread
{
    Q_OBJECT

public:
    PCDFilterThread();
    ~PCDFilterThread();

    int initData(const QString &fileNameDir, const QString& fileSuffix, const int flag);

    void startThread();
    void stopThread();

signals:
    void signalFinish(QString name);

public slots:

protected:
    void run();

private:

    DirProcess dirProcess;
    pcl::PCDReader pcdReader;

    QString dirName;
    QString suffix;
    int flag;

    bool isStart;

    void init();
    bool myMakeDir(const QString& pathDir);

    void removePCDFile(const QString &pcdPath);
};

#endif // PCDFILTERTHREAD_H
