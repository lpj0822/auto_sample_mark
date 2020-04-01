#include "pcdconverterthread.h"
#include <iostream>
#include <QFileInfo>
#include <QDir>

PCDConverterThread::PCDConverterThread()
{
    init();
}

PCDConverterThread::~PCDConverterThread()
{

}

void PCDConverterThread::run()
{
    if (isStart)
    {
        QString saveDirName = this->dirName + "/bins";
        QList<QString> pcdList = dirProcess.getDirFileName(this->dirName, suffix);
        myMakeDir(saveDirName);
        saveDirName += "/";
        for (int i = 0; i< pcdList.size(); ++i)
        {
            if (!isStart)
            {
                break;
            }
            QFileInfo fileInfo(pcdList[i]);
            QString fileName = fileInfo.completeBaseName();
            QString saveFileName = saveDirName + fileName + format;
            pcl::PCLPointCloud2::Ptr srcCloud(new pcl::PCLPointCloud2);
            if(pcdReader.read(pcdList[i].toStdString(), *srcCloud) < 0)
            {
                continue;
            }
            pcWriter.savePointCloudToBin(srcCloud, saveFileName.toStdString(),
                                         this->fieldsNumber);
        }
        emit signalFinish(saveDirName);
    }
}

int PCDConverterThread::initData(const QString &fileNameDir, const QString& fileSuffix,
                                 const QString& format, int fieldsNumber)
{
    this->dirName = fileNameDir;
    this->suffix = fileSuffix;
    this->format = format;
    this->fieldsNumber = fieldsNumber;
    return 0;
}

//开始线程
void PCDConverterThread::startThread()
{
    isStart = true;
}

//结束线程
void PCDConverterThread::stopThread()
{
    isStart = false;
}

bool PCDConverterThread::myMakeDir(const QString& pathDir)
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

void PCDConverterThread::init()
{
    dirName = "";
    suffix = "";
    format = "";
    fieldsNumber = 3;
    isStart = false;
}
