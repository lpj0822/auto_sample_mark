#pragma execution_character_set("utf-8")
#include "croppingvideothread.h"
#include <QHBoxLayout>
#include <QFileDialog>
#include <QTime>
#include <QDebug>
#include <iostream>
#include "math.h"

CroppingVideoThread::CroppingVideoThread()
{
    init();
}

CroppingVideoThread::~CroppingVideoThread()
{
    if(outputVideo)
    {
        delete outputVideo;
        outputVideo = NULL;
    }
}

void CroppingVideoThread::run()
{
    cv::Mat frame;
    int errCode=0;
    while (isStart)
    {
        if(videoProcess->isOpen())
        {
            if(videoProcess->getFramePosition() <= stopFrame)
            {
                errCode=videoProcess->readFrame(frame);
                if(errCode==0)
                {
                    if (isScale)
                    {
                        cv::resize(frame, frame,cv::Size(),scaleSize,scaleSize);
                    }
                    outputVideo->saveVideo(frame);
                }
                else
                {
                    stopThread();
                    emit signalFinish(fileName);
                }
            }
            else
            {
                stopThread();
                emit signalFinish(fileName);
            }
        }
        else
        {
            stopThread();
        }
    }
    outputVideo->closeWriteVideo();
}

//初始化保存数据参数
int CroppingVideoThread::initSaveCroppingVideoData(const QString &fileNameDir, std::shared_ptr<VideoProcess> video,
                                                   double scale, bool isAll, int startPos, int stopPos)
{
    QDir makeDir;
    int errCode=0;
    outputVideo->closeWriteVideo();
    if(video->isOpen())
    {
        if(isAll)
        {
            this->startFrame = 0;
            this->stopFrame=video->getFrameCount();
            video->setFramePosition(0);
            this->videoProcess=video;
            this->fileName=fileNameDir+"/video"+QTime::currentTime().toString("hhmmsszzz")+".avi";
            if(!makeDir.exists(fileNameDir))
            {
                if(!makeDir.mkdir(fileNameDir))
                {
                    std::cout<<"make dir fail!"<<std::endl;
                    return -11;
                }
            }
            std::cout<<"fileName:"<<fileName.toStdString()<<std::endl;
        }
        else
        {
            this->startFrame=startPos*video->getFrameFPS();
            this->stopFrame=stopPos*video->getFrameFPS();
            video->setFramePosition(startFrame);
            this->videoProcess=video;
            this->fileName=fileNameDir+"/video"+QTime::currentTime().toString("hhmmsszzz")+".avi";
            if(!makeDir.exists(fileNameDir))
            {
                if(!makeDir.mkdir(fileNameDir))
                {
                    std::cout<<"make dir fail!"<<std::endl;
                    return -11;
                }
            }
            std::cout<<"fileName:"<<fileName.toStdString()<<std::endl;
        }
        isScale = true;
        this->scaleSize = scale;
        this->fps = video->getFrameFPS();
        this->size = cv::Size(video->getSize().width*scaleSize,video->getSize().height*scaleSize);
        errCode = outputVideo->initSaveVideoData(fileName.toStdString().c_str(),size,fps,codec,isColor);
        return errCode;
    }
    else
    {
        return -1;
    }
}

int CroppingVideoThread::initSaveCroppingVideoData(const QString &fileNameDir, QString &saveFileName,
                                                   std::shared_ptr<VideoProcess> video, int startPos, int stopPos)
{
    QDir makeDir;
    int errCode=0;
    stopThread();
    if(video->isOpen())
    {
        this->startFrame = startPos * video->getFrameFPS();
        this->stopFrame = stopPos * video->getFrameFPS();
        video->setFramePosition(startFrame);
        this->videoProcess = video;
        this->fileName = fileNameDir + "/" + saveFileName;
        if(!makeDir.exists(fileNameDir))
        {
            if(!makeDir.mkdir(fileNameDir))
            {
                std::cout << "make dir fail!" << std::endl;
                return -11;
            }
        }
        std::cout << "fileName:" << fileName.toStdString() << std::endl;

        this->fps = video->getFrameFPS();
        this->size = cv::Size(video->getSize().width,video->getSize().height);
        errCode = outputVideo->initSaveVideoData(fileName.toStdString().c_str(),size,fps,codec,isColor);
        return errCode;
    }
    else
    {
        return -1;
    }
}

int CroppingVideoThread::setNextCroppingVideoData(const QString &fileNameDir, QString &saveFileName, int stopPos)
{
    QDir makeDir;
    int errCode=0;
    stopThread();
    if(videoProcess->isOpen())
    {
        this->stopFrame = stopPos * videoProcess->getFrameFPS();
        this->fileName = fileNameDir + "/" + saveFileName;
        if(!makeDir.exists(fileNameDir))
        {
            if(!makeDir.mkdir(fileNameDir))
            {
                std::cout << "make dir fail!" << std::endl;
                return -11;
            }
        }
        std::cout << "fileName:" << fileName.toStdString() << std::endl;

        errCode = outputVideo->initSaveVideoData(fileName.toStdString().c_str(), size, fps, codec, isColor);
        return errCode;
    }
    else
    {
        return -1;
    }
}

//开始线程
void CroppingVideoThread::startThread()
{
    isStart=true;
}

//结束线程
void CroppingVideoThread::stopThread()
{
    isStart=false;
}

void CroppingVideoThread::init()
{
    videoProcess = nullptr;

    outputVideo = new MyVideoWriter();

    fileName = "";

    size=cv::Size(640,480);//保存视频文件的大小
    fps = 25;//帧率
    codec = cv::VideoWriter::fourcc('X','V','I','D');//视频编码格式
    isColor = true;//是否是彩色

    startFrame = 0;
    stopFrame = 0;

    isScale = false;
    scaleSize = 1.0;

    isStart = false;
}
