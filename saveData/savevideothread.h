#ifndef SAVEVIDEOTHREAD_H
#define SAVEVIDEOTHREAD_H

#include <QThread>
#include <QList>
#include <QString>
#include "myvideowriter.h"
#include "helpers/convertcvqimage.h"

class SaveVideoThread : public QThread
{
    Q_OBJECT

public:
    SaveVideoThread();
    ~SaveVideoThread();

    int initSaveVideoData(QList<QString> images,const QString &fileNameDir, const QString &fileName,int fps=25);

    void startThread();//开始线程
    void stopThread();//结束线程

    void closeVideo();

signals:
    void signalFinish(QString name);

public slots:

protected:
    void run();

private:

    ConvertCVQImage convertImage;
    MyVideoWriter *outputVideo;

    QList<QString> imageList;

    QString fileName;//保存的文件名

    bool isStart;//是否允许线程

    cv::Size size;//保存视频文件的大小
    double fps;//帧率
    int codec;//视频编码格式
    bool isColor;//是否是彩色

    void init();
};

#endif // SAVEVIDEOTHREAD_H
