#include "segmentationlabelconvertthread.h"
#include <QDir>
#include <QImageReader>
#include <QDebug>
#include <iostream>

SegmentationLabelConvertThread::SegmentationLabelConvertThread()
{
    init();
}

SegmentationLabelConvertThread::~SegmentationLabelConvertThread()
{

}

int SegmentationLabelConvertThread::initData(const QString &fileNameDir)
{
    this->dirPath = fileNameDir;
    return 0;
}

void SegmentationLabelConvertThread::startThread()
{
    isStart = true;
}

void SegmentationLabelConvertThread::stopThread()
{
    isStart = false;
}

void SegmentationLabelConvertThread::run()
{
    if (isStart)
    {
        QList<QString> processDataList;
        processDataList.clear();
        QString saveSegmentationDir = this->dirPath + "/../" + "SegmentLabel";
        QString saveAnnotationsDir = this->dirPath + "/../" + "Annotations";
        processDataList = dirProcess.getDirFileName(this->dirPath);
        myMakeDir(saveSegmentationDir);
        for (int i = 0; i< processDataList.size(); ++i)
        {
            if (!isStart)
            {
                break;
            }
            QImageReader reader(processDataList[i]);
            reader.setDecideFormatFromContent(true);
            if(reader.canRead())
            {
                QImage image;
                if(reader.read(&image))
                {
                    QFileInfo fileInfo(processDataList[i]);
                    QString fileName = fileInfo.completeBaseName();
                    QString saveImagePath = saveSegmentationDir + "/" + fileName + ".png";
                    QString readJsonPath= saveAnnotationsDir + "/" + fileName + ".json";
                    QFileInfo jsonFileInfo(readJsonPath);
                    QList<MyObject> objects;
                    if(jsonFileInfo.exists() && jsonProcess.readJSON(readJsonPath, objects) == 0)
                    {
                        QImage mask = maskProcess.createSegmentMask(objects, image.width(), image.height());
                        if(!mask.isNull())
                        {
                            if(!mask.save(saveImagePath))
                            {
                                qDebug() << "save segment mask failed...." << saveImagePath;
                            }
                        }
                    }
                }
                else
                {
                    qDebug() << "open is failed...." << reader.errorString();
                }
            }
            else
            {
                qDebug() << "can not read...." << reader.errorString();
            }
        }
        emit signalFinish(saveSegmentationDir);
    }
}

bool SegmentationLabelConvertThread::myMakeDir(const QString& pathDir)
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

void SegmentationLabelConvertThread::init()
{
    dirPath = "";
    isStart = false;
}

