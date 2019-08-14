#include "imageconverterthread.h"
#include <iostream>
#include <QFileInfo>
#include <QDir>

ImageConverterThread::ImageConverterThread()
{
    init();
}

ImageConverterThread::~ImageConverterThread()
{
    if(dirProcess != NULL)
    {
        delete dirProcess;
        dirProcess = NULL;
    }
    if(imageWriter != NULL)
    {
        delete imageWriter;
        imageWriter = NULL;
    }
}

void ImageConverterThread::run()
{
    if (isStart)
    {
        QString saveDirName = this->dirName + "/temp";
        QList<QString> imageList = dirProcess->getDirFileName(this->dirName, suffix);
        myMakeDir(saveDirName);
        saveDirName += "/";
        for (int i = 0; i< imageList.size(); ++i)
        {
            if (!isStart)
            {
                break;
            }
            QFileInfo fileInfo(imageList[i]);
            QString fileName = fileInfo.completeBaseName();
            QString saveFileName = saveDirName + fileName + imageFormat;
            cv::Mat image = cv::imread(imageList[i].toStdString());
            imageWriter->saveImage(image, saveFileName.toStdString());
        }
        emit signalFinish(saveDirName);
    }
}

int ImageConverterThread::initImageData(const QString &fileNameDir, const QString& fileSuffix, const QString& imageFormat)
{
    this->dirName = fileNameDir;
    this->suffix = fileSuffix;
    this->imageFormat = imageFormat;
    return 0;
}

//开始线程
void ImageConverterThread::startThread()
{
    isStart = true;
}

//结束线程
void ImageConverterThread::stopThread()
{
    isStart = false;
}

bool ImageConverterThread::myMakeDir(const QString& pathDir)
{
    QDir dir;
    if (!dir.exists(pathDir))
    {
        if (!dir.mkpath(pathDir))
        {
            std::cout << "make dir fail!" << "fileName:" << pathDir.toStdString() << std::endl;
            return false;
        }
    }
    return true;
}

void ImageConverterThread::init()
{
    dirProcess = new DirProcess();
    imageWriter = new MyImageWriter();

    dirName = "";//保存的文件名
    suffix = "";//图片后缀
    imageFormat = "";//转换成的图片格式

    isStart = false;
}
