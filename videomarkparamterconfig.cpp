#pragma execution_character_set("utf-8")
#include "videomarkparamterconfig.h"
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QFile>
#include <QFileInfo>
#include <QVariant>

int VideoMarkParamterConfig::skipFrameNumber = 1;
bool VideoMarkParamterConfig::isTracking = true;
TrackingMethod VideoMarkParamterConfig::trackingMethod = TrackingMethod::KALMAN;

VideoMarkParamterConfig::VideoMarkParamterConfig()
{
    init();
}

VideoMarkParamterConfig::~VideoMarkParamterConfig()
{

}

void VideoMarkParamterConfig::setSkipFrameNumber(int number)
{
    skipFrameNumber = number;
}

void VideoMarkParamterConfig::setIsTracking(bool whether)
{
    isTracking = whether;
}

void VideoMarkParamterConfig::setTrackingMethod(int method)
{
    switch(method)
    {
    case -1:
        trackingMethod = TrackingMethod::UNTRACKINGMETHOD;
        break;
    case 0:
        trackingMethod = TrackingMethod::KALMAN;
        break;
    case 1:
        trackingMethod = TrackingMethod::KCF;
        break;
    case 2:
        trackingMethod = TrackingMethod::TLD;
        break;
    case 3:
        trackingMethod = TrackingMethod::MIL;
        break;
    }
}

int VideoMarkParamterConfig::getSkipFrameNumber()
{
    return skipFrameNumber;
}

bool VideoMarkParamterConfig::getIsTracking()
{
    return isTracking;
}

TrackingMethod VideoMarkParamterConfig::getTrackingMethod()
{
    return trackingMethod;
}

int VideoMarkParamterConfig::loadConfig()
{
    QByteArray data;
    QFile file;
    file.setFileName("./video_mark_config.json");
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
                    skipFrameNumber = jsonObject.take("skipFrameNumber").toVariant().toInt();
                }
                if(jsonObject.contains("isTracking"))
                {
                    int result = jsonObject.take("isTracking").toVariant().toInt();
                    if(result)
                    {
                        isTracking = true;
                    }
                    else
                    {
                        isTracking = false;
                    }
                }
                if(jsonObject.contains("trackingMethod"))
                {
                    int result = jsonObject.take("trackingMethod").toVariant().toInt();
                    switch(result)
                    {
                    case -1:
                        trackingMethod = TrackingMethod::UNTRACKINGMETHOD;
                        break;
                    case 0:
                        trackingMethod = TrackingMethod::KALMAN;
                        break;
                    case 1:
                        trackingMethod = TrackingMethod::KCF;
                        break;
                    case 2:
                        trackingMethod = TrackingMethod::TLD;
                        break;
                    case 3:
                        trackingMethod = TrackingMethod::MIL;
                        break;
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

int VideoMarkParamterConfig::saveConfig()
{
    QJsonDocument doc;
    QByteArray data;
    QJsonObject jsonData;
    QFile file("./video_mark_config.json");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate |QIODevice::Text))
    {
        return -1;
    }
    jsonData.insert("skipFrameNumber", QString::number(skipFrameNumber));
    jsonData.insert("isTracking", QString::number((int)isTracking));
    jsonData.insert("trackingMethod", QString::number(trackingMethod));

    doc.setObject(jsonData);
    data = doc.toJson();
    file.write(data);
    file.close();
    return 0;
}

void VideoMarkParamterConfig::init()
{

}


