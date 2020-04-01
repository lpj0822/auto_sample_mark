#ifndef PCDCONVERTERTHREAD_H
#define PCDCONVERTERTHREAD_H

#include <QThread>
#include "helpers/dirprocess.h"
#include "saveData/pointcloudwriter.h"

class PCDConverterThread : public QThread
{
    Q_OBJECT

public:
    PCDConverterThread();
    ~PCDConverterThread();

    int initData(const QString &fileNameDir, const QString& fileSuffix,
                 const QString& format, int fieldsNumber);

    void startThread();//开始线程
    void stopThread();//结束线程

signals:
    void signalFinish(QString name);

public slots:

protected:
    void run();

private:

    DirProcess dirProcess;
    PointCloudWriter pcWriter;
    pcl::PCDReader pcdReader;

    QString dirName;
    QString suffix;
    QString format;
    int fieldsNumber;

    bool isStart;

    void init();
    bool myMakeDir(const QString& pathDir);
};

#endif // PCDCONVERTERTHREAD_H
