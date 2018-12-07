#pragma execution_character_set("utf-8")
#include "saveimagethread.h"
#include <QDir>
#include <QImage>
#include <QDebug>

SaveImageThread::SaveImageThread()
{
    init();
}

SaveImageThread::~SaveImageThread()
{

}

void SaveImageThread::run()
{
    int frameNumber = 0;
    cv::Mat frame;
    QImage currentImage;
    int errCode=0;
    QString fileName;
    while (isStart)
    {
        if(videoProcess->isOpen())
        {
            frameNumber = videoProcess->getFramePosition();
            if(frameNumber <= stopFrame)
            {
                errCode = videoProcess->readFrame(frame);
                if(errCode==0)
                {
                    if(frameNumber % skipFrameNuber == 0)
                    {

                        fileName = dirName + "/" + QString("/%1_%2").arg(this->videoFileName).arg(frameNumber)
                                + imagePost;
                        //myImageWrite.saveImage(frame, fileName.toStdString());
                        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
                        currentImage = QImage((uchar*)(frame.data), frame.cols, frame.rows,
                                              QImage::Format_RGB888);
                        currentImage.save(fileName);
                        emit signalCurrentFrame(frameNumber);
                    }
                }
                else
                {
                    isStart = false;
                    emit signalFinish(dirName);
                }
            }
            else
            {
                isStart = false;
                emit signalFinish(dirName);
            }
        }
    }
    emit signalCurrentFrame(0);
    videoProcess->closeVideo();
}

//初始化保存数据参数
int SaveImageThread::initSaveImageData(const QString &videoPath, QString imagePost,
                                       std::shared_ptr<VideoProcess> video, int skipFrameNuber,
                                       bool isAll, int startPos, int stopPos)
{
    QDir makeDir;
    QFileInfo info(videoPath);
    this->videoFileName = info.completeBaseName();
    if(video->isOpen())
    {
        if(isAll)
        {
            this->skipFrameNuber = skipFrameNuber;
            this->startFrame = 0;
            this->stopFrame = video->getFrameCount();
            video->setFramePosition(0);
            this->videoProcess = video;
            this->dirName = info.absolutePath() + "/" + this->videoFileName + "_MultipleTarget";
            if(!makeDir.exists(dirName))
            {
                if(!makeDir.mkdir(dirName))
                {
                    qDebug() << "make dir fail!" << endl;
                    return -11;
                }
            }
            qDebug() << "fileName:" << dirName << endl;
        }
        else
        {
            this->skipFrameNuber = skipFrameNuber;
            this->startFrame = startPos * video->getFrameFPS();
            this->stopFrame = stopPos * video->getFrameFPS();
            video->setFramePosition(startFrame);
            this->videoProcess = video;
            this->dirName = info.absolutePath() + "/" + this->videoFileName + "_MultipleTarget";
            if(!makeDir.exists(dirName))
            {
                if(!makeDir.mkdir(dirName))
                {
                    qDebug() << "make dir fail!" << endl;
                    return -11;
                }
            }
            qDebug() << "fileName:" << dirName << endl;
        }
        this->imagePost = imagePost;
        this->widthStr = QString::number(stopFrame).length();
        return 0;
    }
    else
    {
        return -1;
    }
}

//开始线程
void SaveImageThread::startThread()
{
    isStart = true;
}

//结束线程
void SaveImageThread::stopThread()
{
    isStart = false;
}

void SaveImageThread::init()
{
    videoProcess=nullptr;

    videoFileName = "";
    dirName = "";//保存的文件名
    imagePost = ".png";

    skipFrameNuber = 1;
    startFrame = 0;
    stopFrame = 0;

    isStart = false;

    widthStr = 1;
}
