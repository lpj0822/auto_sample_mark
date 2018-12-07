#include "savevideothread.h"
#include <QDir>
#include <iostream>

SaveVideoThread::SaveVideoThread()
{
    init();
}

SaveVideoThread::~SaveVideoThread()
{
    if(outputVideo)
    {
        delete outputVideo;
        outputVideo=NULL;
    }
}

void SaveVideoThread::run()
{
    int loop=0;
    QImage image;
    int count=imageList.size();
    while(isStart)
    {
        if(image.load(imageList[loop]))
        {
            image = image.convertToFormat(QImage::Format_RGB888);
            cv::Mat frame = convertImage.QImageTocvMat(image);
            outputVideo->saveVideo(frame);
        }
        loop++;
        if(loop>=count)
        {
            emit signalFinish(fileName);
            isStart=false;
        }
    }
    outputVideo->closeWriteVideo();
}

//初始化保存数据参数
int SaveVideoThread::initSaveVideoData(QList<QString> images,const QString &fileNameDir, const QString &fileName,int fps)
{
    QImage tempImage;
    int errCode=0;
    QDir makeDir;
    imageList.clear();
    this->fileName=fileName;
    this->imageList=images;
    if(images.size()>=1)
    {
        if(tempImage.load(images[0]))
        {
            size.width=tempImage.width();
            size.height=tempImage.height();
            this->fps=fps;
        }
        else
        {
            return -20;
        }
    }
    else
    {
        return -20;
    }

    if(!makeDir.exists(fileNameDir))
    {
        if(!makeDir.mkdir(fileNameDir))
        {
            std::cout<<"make dir fail!"<<std::endl;
            return -11;
        }
    }
    std::cout<<"fileName:"<<fileName.toStdString()<<std::endl;
    outputVideo->closeWriteVideo();
    errCode=outputVideo->initSaveVideoData(fileName.toStdString().c_str(),size,fps,codec,isColor);
    return errCode;
}

//开始线程
void SaveVideoThread::startThread()
{
    isStart=true;
}

//结束线程
void SaveVideoThread::stopThread()
{
    isStart=false;
}

void SaveVideoThread::closeVideo()
{
    outputVideo->closeWriteVideo();
}

void SaveVideoThread::init()
{
    outputVideo=new MyVideoWriter();

    fileName="";
    imageList.clear();

    size=cv::Size(640,480);//保存视频文件的大小
    fps=25;//帧率
    codec=cv::VideoWriter::fourcc('X','V','I','D');//视频编码格式
    isColor=true;//是否是彩色

    isStart=false;
}
