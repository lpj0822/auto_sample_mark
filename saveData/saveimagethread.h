#ifndef SAVEIMAGETHREAD_H
#define SAVEIMAGETHREAD_H

#include <QThread>
#include <QString>
#include <memory>
#include "myimagewriter.h"
#include "helpers/videoprocess.h"
#include "helpers/dirprocess.h"

class SaveImageThread : public QThread
{
     Q_OBJECT

public:
    SaveImageThread();
    ~SaveImageThread();

    int initSaveImageData(const QString &videoPath, QString imagePost,
                          std::shared_ptr<VideoProcess> video, int skipFrameNuber,
                          bool isAll=true, int startPos=0, int stopPos=0);

    void startThread();//开始线程
    void stopThread();//结束线程

signals:
    void signalFinish(QString name);
    void signalCurrentFrame(int number);

public slots:

protected:
    void run();

private:
    MyImageWriter myImageWrite;
    std::shared_ptr<VideoProcess> videoProcess;
    DirProcess dirProcess;

    QString videoFileName;
    QString dirName;//保存的文件名
    QString imagePost;//文件扩展名

    int skipFrameNuber;
    int startFrame;
    int stopFrame;

    bool isStart;//是否允许线程

    int widthStr;

    void init();
};

#endif // SAVEIMAGETHREAD_H
