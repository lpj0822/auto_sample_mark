#include "pcdfilterterthread.h"
#include <iostream>
#include <QFileInfo>
#include <QFile>
#include <QDir>
#include <QDebug>

PCDFilterThread::PCDFilterThread()
{
    init();
}

PCDFilterThread::~PCDFilterThread()
{

}

void PCDFilterThread::run()
{
    if (isStart)
    {
        QList<QString> pcdList = dirProcess.getDirFileName(this->dirName, suffix);
        for (int i = 0; i< pcdList.size(); ++i)
        {
            if (!isStart)
            {
                break;
            }
            if(this->flag == 0)
            {
                removePCDFile(pcdList[i]);
            }
        }
        emit signalFinish(this->dirName);
    }
}

int PCDFilterThread::initData(const QString &fileNameDir, const QString& fileSuffix, const int flag)
{
    this->dirName = fileNameDir;
    this->suffix = fileSuffix;
    this->flag = flag;
    return 0;
}

void PCDFilterThread::startThread()
{
    isStart = true;
}

void PCDFilterThread::stopThread()
{
    isStart = false;
}

bool PCDFilterThread::myMakeDir(const QString& pathDir)
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

void PCDFilterThread::removePCDFile(const QString &pcdPath)
{
    pcl::PCLPointCloud2::Ptr srcCloud(new pcl::PCLPointCloud2);
    try {
        if(pcdReader.read(pcdPath.toStdString(), *srcCloud) < 0)
        {
            if(!QFile::remove(pcdPath))
            {
                qDebug() << "remove file " << pcdPath << " faild!";
            }
        }
    } catch (std::exception e) {
       std::cout << e.what() << std::endl;
       if(!QFile::remove(pcdPath))
       {
           qDebug() << "remove file " << pcdPath << " faild!";
       }
    }
}

void PCDFilterThread::init()
{
    dirName = "";
    suffix = "";
    isStart = false;
}
