#ifndef AUTOSAMPLEMARK_H
#define AUTOSAMPLEMARK_H

#include <QThread>
#include <QList>
#include <QString>

#include <memory>

#include "helpers/videoprocess.h"
#include "saveMarkData/xmlprocess.h"
#include "deepLearning/ssdector.h"
#include "dataType/myobject.h"

class AutoSampleMarkThread : public QThread
{
    Q_OBJECT

public:
    AutoSampleMarkThread();
    ~AutoSampleMarkThread();

    int initModel(const QString modelConfiguration, const QString modelBinary);
    void initData(const QList<QString> videoList, const int skipFrameCount, const float confidenceThreshold);

    void startThread();//开始线程
    void stopThread();//结束线程

signals:
    void signalCurrentFrame(int number);
    void signalVideoInit(int count);
    void signalVideoFinish(QString name);
    void signalFinishAll();

public slots:

protected:
    void run();

private:

    void markVideo(const QString& videoPath, QString& processInfo);
    void saveImage(const QString& imagePath, const cv::Mat &frame);

    QList<MyObject> toMyObjects(const std::vector<cv::Rect> &objectRect,
                     const std::vector<std::string> &objectClass);

private:

    std::shared_ptr<VideoProcess> videoProcess;
    std::shared_ptr<SSDector> detector;

    XMLProcess xmlCreator;

    bool isStart;
    int skipFrameCount;
    int videoWidth;
    int videoHeight;

    QList<QString> videoList;

    void init();
};

#endif // AUTOSAMPLEMARK_H
