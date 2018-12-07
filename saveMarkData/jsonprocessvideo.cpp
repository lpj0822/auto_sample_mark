#pragma execution_character_set("utf-8")
#include "jsonprocessvideo.h"
#include <QJsonParseError>
#include <QFile>
#include <QFileInfo>
#include <QVariant>

JSONProcessVideo::JSONProcessVideo(QObject *parent) : JSONProcess(parent)
{

}

JSONProcessVideo::~JSONProcessVideo()
{

}

int JSONProcessVideo::createJSON(const QString &jsonFilePath, const QString &videoFilePath, const QMap<int, QList<MyObject> > &result,
                            const int skipNumber)
{
    int maxFrameNumber = -1;
    QJsonDocument doc;
    QByteArray data;
    QJsonObject jsonData;
    QJsonObject allFrameData;
    QFileInfo fileInfo(videoFilePath);
    QFile file(jsonFilePath);
    QMapIterator<int, QList<MyObject> > iter(result);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate |QIODevice::Text))
    {
        return -1;
    }
    jsonData.insert("annotation", "Annotations");
    jsonData.insert("folder", "MultipleTarget");
    jsonData.insert("videoName", fileInfo.fileName());
    jsonData.insert("skipFrameNumber", skipNumber);
    while(iter.hasNext())
    {
        iter.next();
        const int key = iter.key();
        const QList<MyObject>& objects = iter.value();
        QJsonObject frameData;
        writeRectData(objects, frameData);
        writeLineData(objects, frameData);
        writePolygonData(objects, frameData);
        allFrameData.insert(QString::number(key), frameData);
        if(key > maxFrameNumber)
        {
            maxFrameNumber = key;
        }
    }
    jsonData.insert("maxFrameNumber", maxFrameNumber);
    jsonData.insert("objects", allFrameData);
    doc.setObject(jsonData);
    data = doc.toJson();
    file.write(data);
    file.close();
    return 0;
}

int JSONProcessVideo::readJSON(const QString &jsonFilePath, QMap<int, QList<MyObject> > &result, int &skipNumber)
{
    QByteArray data;
    QFile file;
    file.setFileName(jsonFilePath);
    result.clear();
    if(!file.open(QFile::ReadOnly | QFile::Text))
    {
        return -1;
    }
    data = file.readAll();
    file.close();
    QJsonParseError jsonError;
    QJsonDocument parseDoucment = QJsonDocument::fromJson(QString(data).toUtf8(), &jsonError);
    if(jsonError.error == QJsonParseError::NoError)
    {
        if (!(parseDoucment.isNull() || parseDoucment.isEmpty()))
        {
            if (parseDoucment.isObject())
            {
                QJsonObject jsonObject = parseDoucment.object();
                if(jsonObject.contains("skipFrameNumber"))
                {
                    skipNumber = jsonObject.take("skipFrameNumber").toVariant().toInt();
                }
                if(jsonObject.contains("objects"))
                {
                    QJsonObject allFrameData = jsonObject.take("objects").toObject();
                    QJsonObject::const_iterator iter;
                    for(iter = allFrameData.begin(); iter != allFrameData.end(); iter++)
                    {
                        int key = iter.key().toInt();
                        QList<MyObject> objects;
                        objects.clear();
                        QJsonObject frameValue = iter.value().toObject();
                        if(frameValue.contains("rectObject"))
                        {
                            QJsonArray rectValue = frameValue.take("rectObject").toArray();
                            readRectData(rectValue, objects);
                        }
                        if(frameValue.contains("lineObject"))
                        {
                            QJsonArray lineValue = frameValue.take("lineObject").toArray();
                            readLineData(lineValue, objects);
                        }
                        if(frameValue.contains("polygonObject"))
                        {
                            QJsonArray polygonValue = frameValue.take("polygonObject").toArray();
                            readPolygonData(polygonValue, objects);
                        }
                        result[key] = objects;
                    }
                }
            }
        }
    }
    else
    {
        return -2;
    }
    return 0;
}
