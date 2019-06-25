#pragma execution_character_set("utf-8")
#include "autosamplemarkthread.h"
#include <QFileInfo>
#include <QDir>
#include <QImage>

#include <iostream>

#include "sampleMarkParam/manualparamterconfig.h"
#include "autoparamterconfig.h"

AutoSampleMarkThread::AutoSampleMarkThread()
{
    init();
}

AutoSampleMarkThread::~AutoSampleMarkThread()
{

}

int AutoSampleMarkThread::initModel(const QString modelConfiguration, const QString modelBinary)
{
    int errorCode = 0;
    errorCode = detector->initModel(modelConfiguration.toStdString(),
                                    modelBinary.toStdString());
    return errorCode;
}

void AutoSampleMarkThread::initData(const QList<QString> videoList, const int skipFrameCount, const float confidenceThreshold)
{
    this->videoList = videoList;
    this->skipFrameCount = skipFrameCount;
    this->detector->initDetectorParameters(AutoParamterConfig::getInpuDataWidth(), AutoParamterConfig::getInpuDataHeight(),
                                           confidenceThreshold, AutoParamterConfig::getModelLabels());
}

void AutoSampleMarkThread::startThread()
{
    isStart = true;
}

void AutoSampleMarkThread::stopThread()
{
    isStart = false;
}

void AutoSampleMarkThread::run()
{
    QString processInfo = "";
    int errorCode = 0;
    int count = videoList.size();
    for(int index = 0; index < count; index++)
    {
        errorCode = videoProcess->openVideo(videoList[index]);
        if(errorCode == 0 && videoProcess->isOpen())
        {
            videoWidth = videoProcess->getSize().width;
            videoHeight = videoProcess->getSize().height;
            emit signalVideoInit(videoProcess->getFrameCount());
            markVideo(videoList[index], processInfo);
        }
        else
        {
            processInfo = QString("Error:%1 can not open!").arg(videoList[index]);
        }
        emit signalVideoFinish(processInfo);
        videoProcess->closeVideo();
        if(!isStart)
        {
            break;
        }
    }
    isStart = false;
    emit signalFinishAll();
}

void AutoSampleMarkThread::markVideo(const QString& videoPath, QString& processInfo)
{
    int errorCode = 0;
    cv::Mat frame;
    int stopFrame = videoProcess->getFrameCount();
    int currentFrame = videoProcess->getFramePosition() - 1;
    QFileInfo fileInfo(videoPath);
    QString path = fileInfo.path();
    QString videoName = fileInfo.completeBaseName();
    QString saveImageDir = path + "/" + videoName + "_MultipleTarget";
    QString saveXMLDir = path + "/" + "Annotations";
    QDir makeDir;
    if(!makeDir.exists(saveImageDir))
    {
        if(!makeDir.mkdir(saveImageDir))
        {
            std::cout << "make dir fail!" << std::endl;
        }
    }
    if(!makeDir.exists(saveXMLDir))
    {
        if(!makeDir.mkdir(saveXMLDir))
        {
            std::cout << "make dir fail!" << std::endl;
        }
    }
    while(currentFrame < stopFrame)
    {
        if(isStart)
        {
            errorCode = videoProcess->readFrame(frame);
            if(errorCode == 0)
            {
                if(currentFrame % this->skipFrameCount == 0)
                {
                    std::vector<cv::Rect> objectRect;
                    std::vector<std::string> objectClass;
                    std::vector<float> objectConfidence;
                    QList<MyObject> objects;
                    detector->processDetect(frame, objectRect, objectClass, objectConfidence);
                    QString savXmlPath = saveXMLDir + QString("/%1_%2.xml").arg(videoName).arg(currentFrame);
                    QString saveImagePath = saveImageDir + QString("/%1_%2.png").arg(videoName).arg(currentFrame);
                    objects = toMyObjects(objectRect, objectClass);
                    xmlCreator.createXML(savXmlPath, saveImagePath, videoWidth, videoHeight, objects);
                    saveImage(saveImagePath, frame);
                    emit signalCurrentFrame(currentFrame + 1);
                }
            }
            else
            {
                break;
            }
        }
        else
        {
            break;
        }
        currentFrame = videoProcess->getFramePosition() - 1;
    }

    if((currentFrame + 1) == stopFrame)
    {
        processInfo = "Video:" + videoPath;
    }
    else
    {
        if(isStart)
        {
            processInfo = QString("Error: %1 read frame error!").arg(videoPath);
        }
        else
        {
            processInfo = "Stop!";
        }
    }
}

void AutoSampleMarkThread::saveImage(const QString& imagePath, const cv::Mat &frame)
{
    cv::Mat tempMat;
    if(frame.empty())
    {
        return;
    }
    cv::cvtColor(frame, tempMat, cv::COLOR_BGR2RGB);//Qt中支持的是RGB图像, OpenCV中支持的是BGR
    QImage image = QImage((uchar*)(tempMat.data), tempMat.cols, tempMat.rows, QImage::Format_RGB888);
    if(!image.save(imagePath))
    {
        std::cout << "save image fail!" << std::endl;
    }
}

QList<MyObject> AutoSampleMarkThread::toMyObjects(const std::vector<cv::Rect> &objectRect,
                 const std::vector<std::string> &objectClass)
{
    QList<MyObject> objects;
    int count = static_cast<int>(objectRect.size());
    objects.clear();
    for(int index = 0; index < count; index++)
    {
        MyObject object;
        const cv::Rect cvRect = objectRect[index];
        if(cvRect.width >= ManualParamterConfig::getMinWidth() && cvRect.height >= ManualParamterConfig::getMinHeight())
        {
            object.setBox(QRect(cvRect.x, cvRect.y, cvRect.width, cvRect.height));
            object.setObjectClass(QString::fromStdString(objectClass[index]));
        }
    }
    return objects;
}

void AutoSampleMarkThread::init()
{
    isStart = false;
    skipFrameCount = 1;
    videoWidth = 640;
    videoHeight = 480;

    videoList.clear();

    videoProcess = std::shared_ptr<VideoProcess>(new VideoProcess());
    detector = std::shared_ptr<SSDector>(new SSDector());
}
