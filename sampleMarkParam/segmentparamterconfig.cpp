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
#include <QColor>

#include "manualparamterconfig.h"

int SegmentParamterConfig::LINE_WIDTH = 10;

SegmentParamterConfig::SegmentParamterConfig()
{
    init();
}

SegmentParamterConfig::~SegmentParamterConfig()
{

}

void SegmentParamterConfig::setLineWidth(const int width)
{
    LINE_WIDTH = width;
}

int SegmentParamterConfig::getLineWidth()
{
    return LINE_WIDTH;
}

int SegmentParamterConfig::loadClassConfig(const QString &classPath)
{
    return 0;
}

int SegmentParamterConfig::saveClassConfig(const QString &saveClassPath)
{
    QMap<QString, QString>::const_iterator classIterator;
    QMap<QString, QVariant> saveMarkClass;
    QJsonDocument doc;
    QByteArray data;
    QJsonObject jsonData;
    QFile file(saveClassPath);
    QMap<QString, QString> markClass = ManualParamterConfig::getMarkClass();
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate |QIODevice::Text))
    {
        return -1;
    }
    for(classIterator = markClass.constBegin(); classIterator != markClass.constEnd(); ++classIterator)
    {
        QColor color(classIterator.value());
        jsonData.insert(classIterator.key(), QString("%1,%2,%3").arg(color.red()).arg(color.green()).arg(color.blue()));
    }
    doc.setObject(jsonData);
    data = doc.toJson();
    file.write(data);
    file.close();
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
                if(jsonObject.contains("lineWidth"))
                {
                    LINE_WIDTH = jsonObject.take("lineWidth").toVariant().toInt();
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
    QJsonDocument doc;
    QByteArray data;
    QJsonObject jsonData;
    QFile file("./segment_mark_config.json");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate |QIODevice::Text))
    {
        return -1;
    }
    jsonData.insert("lineWidth", QString::number(LINE_WIDTH));
    doc.setObject(jsonData);
    data = doc.toJson();
    file.write(data);
    file.close();
    return 0;
}

void SegmentParamterConfig::init()
{

}

