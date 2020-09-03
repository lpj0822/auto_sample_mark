#include "pcdconverterthread.h"
#include <iostream>
#include <QFileInfo>
#include <QFile>
#include <QDir>
#include <QDebug>

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
            try {
                if(pcdReader.read(pcdList[i].toStdString(), *srcCloud) < 0)
                {
                    qDebug() << "open " << pcdList[i] << " faild!";
                    continue;
                }
                pcWriter.savePointCloudToBin(srcCloud, saveFileName.toStdString(),
                                             this->fieldsNumber);
            } catch (std::exception e) {
               qDebug() << e.what() << "|open " << pcdList[i] << " faild!";
            }
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

void PCDConverterThread::startThread()
{
    isStart = true;
}

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
