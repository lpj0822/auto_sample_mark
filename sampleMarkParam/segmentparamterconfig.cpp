#pragma execution_character_set("utf-8")
#include "segmentparamterconfig.h"
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QFile>
#include <QFileInfo>
#include <QVariant>

#include "drawmarkcolor.h"

int SegmentParamterConfig::NEAR_POINT_LENGTH = 10;
QMap<QString, QString> SegmentParamterConfig::markClass = QMap<QString, QString>();

SegmentParamterConfig::SegmentParamterConfig()
{
    init();
}

SegmentParamterConfig::~SegmentParamterConfig()
{

}

void SegmentParamterConfig::setNearPointLength(int lenght)
{
    NEAR_POINT_LENGTH = lenght;
}

void SegmentParamterConfig::setMarkClass(QMap<QString, QString> classMark)
{
    markClass.clear();
    markClass = classMark;
}

void SegmentParamterConfig::addMarkClass(QString className, QString classColor)
{
    markClass.insert(className, classColor);
}

void SegmentParamterConfig::removeMarkClass(QString className)
{
    markClass.remove(className);
}

int SegmentParamterConfig::getNearPointLenght()
{
    return NEAR_POINT_LENGTH;
}

QMap<QString, QString> SegmentParamterConfig::getMarkClass()
{
    return markClass;
}

QString SegmentParamterConfig::getMarkClassColor(QString className)
{
    if(markClass.contains(className))
    {
        return markClass[className];
    }
    else
    {
        return "";
    }
}

int SegmentParamterConfig::loadClassConfig(const QString &classPath)
{
    return 0;
}

int SegmentParamterConfig::loadConfig()
{
    QByteArray data;
    QFile file;
    file.setFileName("./segment_mark_config.json");
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
                if(jsonObject.contains("nearPointLength"))
                {
                    NEAR_POINT_LENGTH = jsonObject.take("nearPointLength").toVariant().toInt();
                }

                if(jsonObject.contains("markClass"))
                {
                    markClass.clear();
                    QMap<QString, QVariant> readMarkClass = jsonObject.take("markClass").toObject().toVariantMap();
                    for(QMap<QString, QVariant>::const_iterator iter = readMarkClass.constBegin();
                        iter != readMarkClass.constEnd(); ++iter)
                    {
                        markClass.insert(iter.key(), iter.value().toString());
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

int SegmentParamterConfig::saveConfig()
{
    QMap<QString, QString>::const_iterator classIterator;
    QMap<QString, QVariant> saveMarkClass;
    QJsonDocument doc;
    QByteArray data;
    QJsonObject jsonData;
    QFile file("./segment_mark_config.json");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate |QIODevice::Text))
    {
        return -1;
    }
    for(classIterator = markClass.constBegin(); classIterator != markClass.constEnd(); ++classIterator)
    {
        saveMarkClass.insert(classIterator.key(), QVariant(classIterator.value()));
    }
    jsonData.insert("nearPointLength", QString::number(NEAR_POINT_LENGTH));
    jsonData.insert("markClass", QJsonObject::fromVariantMap(saveMarkClass));

    doc.setObject(jsonData);
    data = doc.toJson();
    file.write(data);
    file.close();
    return 0;
}

void SegmentParamterConfig::init()
{

}

