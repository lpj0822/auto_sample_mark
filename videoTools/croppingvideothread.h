#ifndef CROPPINGVIDEOTHREAD_H
#define CROPPINGVIDEOTHREAD_H

#include <QThread>
#include <QString>
#include <memory>
#include <opencv2/imgproc.hpp>
#include "saveData/myvideowriter.h"
#include "helpers/videoprocess.h"

class CroppingVideoThread : public QThread
{
    Q_OBJECT

public:
    CroppingVideoThread();
    ~CroppingVideoThread();

    int initSaveCroppingVideoData(const QString &fileNameDir, std::shared_ptr<VideoProcess> video,
                                  double scale=1.0, bool isAll=true, int startPos=0, int stopPos=0);

    int initSaveCroppingVideoData(const QString &fileNameDir, QString &saveFileName,
                                  std::shared_ptr<VideoProcess> video, int startPos=0, int stopPos=0);

    int setNextCroppingVideoData(const QString &fileNameDir, QString &saveFileName, int stopPos);

    void startThread();//开始线程
    void stopThread();//结束线程

signals:
    void signalFinish(QString name);

public slots:

protected:
    void run();

private:

    MyVideoWriter *outputVideo;
    std::shared_ptr<VideoProcess> videoProcess;

    bool isScale;
    double scaleSize;//缩放尺寸

    QString fileName;//保存的文件名

    cv::Size size;//保存视频文件的大小
    double fps;//帧率
    int codec;//视频编码格式
    bool isColor;//是否是彩色

    int startFrame;
    int stopFrame;

    bool isStart;//是否允许线程

    void init();
};

#endif // CROPPINGVIDEOTHREAD_H
