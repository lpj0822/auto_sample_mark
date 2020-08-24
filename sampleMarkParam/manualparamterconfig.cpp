#pragma execution_character_set("utf-8")
#include "manualparamterconfig.h"
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QFile>
#include <QFileInfo>
#include <QVariant>
#include <QColor>

#include "drawmarkcolor.h"

int ManualParamterConfig::MIN_WIDTH = 1;
int ManualParamterConfig::MIN_HEIGHT = 1;
int ManualParamterConfig::NEAR_POINT_LENGTH = 5;
int ManualParamterConfig::MAX_SCALE = 300;
int ManualParamterConfig::MIN_SCALE = 50;
QMap<QString, QString> ManualParamterConfig::markClass = QMap<QString, QString>();

ManualParamterConfig::ManualParamterConfig()
{
    init();
}

ManualParamterConfig::~ManualParamterConfig()
{

}

void ManualParamterConfig::setMinWidth(int minWidth)
{
    MIN_WIDTH = minWidth;
}

void ManualParamterConfig::setMinHeight(int minHeight)
{
    MIN_HEIGHT = minHeight;
}

void ManualParamterConfig::setNearPointLength(int lenght)
{
    NEAR_POINT_LENGTH = lenght;
}

void ManualParamterConfig::setMinScale(int minScale)
{
    MIN_SCALE = minScale;
}

void ManualParamterConfig::setMaxScale(int maxScale)
{
    MAX_SCALE = maxScale;
}

void ManualParamterConfig::setMarkClass(QMap<QString, QString> classMark)
{
    markClass.clear();
    markClass = classMark;
}

void ManualParamterConfig::addMarkClass(QString className, QString classColor)
{
    markClass.insert(className, classColor);
}

void ManualParamterConfig::removeMarkClass(QString className)
{
    markClass.remove(className);
}

int ManualParamterConfig::getMinWidth()
{
    return MIN_WIDTH;
}

int ManualParamterConfig::getMinHeight()
{
    return MIN_HEIGHT;
}

int ManualParamterConfig::getMinSacle()
{
    return MIN_SCALE;
}

int ManualParamterConfig::getMaxScale()
{
    return MAX_SCALE;
}

int ManualParamterConfig::getNearPointLenght()
{
    return NEAR_POINT_LENGTH;
}

QMap<QString, QString> ManualParamterConfig::getMarkClass()
{
    return markClass;
}

QString ManualParamterConfig::getMarkClassColor(QString className)
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

int ManualParamterConfig::loadConfig()
{
    QByteArray data;
    QFile file;
    file.setFileName("./manual_mark_config.json");
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
                if(jsonObject.contains("minWidth"))
                {
                    MIN_WIDTH = jsonObject.take("minWidth").toVariant().toInt();
                }

                if(jsonObject.contains("minHeight"))
                {
                    MIN_HEIGHT = jsonObject.take("minHeight").toVariant().toInt();
                }

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

int ManualParamterConfig::saveConfig()
{
    QMap<QString, QString>::const_iterator classIterator;
    QMap<QString, QVariant> saveMarkClass;
    QJsonDocument doc;
    QByteArray data;
    QJsonObject jsonData;
    QFile file("./manual_mark_config.json");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate |QIODevice::Text))
    {
        return -1;
    }
    for(classIterator = markClass.constBegin(); classIterator != markClass.constEnd(); ++classIterator)
    {
        saveMarkClass.insert(classIterator.key(), QVariant(classIterator.value()));
    }
    jsonData.insert("minWidth", QString::number(MIN_WIDTH));
    jsonData.insert("minHeight", QString::number(MIN_HEIGHT));
    jsonData.insert("nearPointLength", QString::number(NEAR_POINT_LENGTH));
    jsonData.insert("markClass", QJsonObject::fromVariantMap(saveMarkClass));

    doc.setObject(jsonData);
    data = doc.toJson();
    file.write(data);
    file.close();
    return 0;
}

int ManualParamterConfig::loadDetClassConfig(const QString &classPath)
{
    QByteArray data;
    QFile file;
    file.setFileName(classPath);
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
                if(jsonObject.contains("DetectionClass"))
                {
                    QMap<QString, QVariant> readMarkClass = jsonObject.take("DetectionClass").toObject().toVariantMap();
                    int index = 0;
                    QString color = "#000000";
                    markClass.clear();
                    for(QMap<QString, QVariant>::const_iterator iter = readMarkClass.constBegin();
                        iter != readMarkClass.constEnd(); ++iter)
                    {
                        index = iter.key().toInt();
                        color = drawMarkColor[index % DRAW_MARK_COLOR_COUNT];
                        markClass.insert(iter.value().toString(), color);
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

int ManualParamterConfig::saveDetClassConfig(const QString &classPath)
{
    if(markClass.count() == 0)
        return -2;
    QMap<QString, QString>::const_iterator classIterator;
    QJsonDocument doc;
    QByteArray data;
    QJsonObject jsonData;
    QJsonObject classData;
    QFile file(classPath);
    int index = 0;
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate |QIODevice::Text))
    {
        return -1;
    }
    for(classIterator = markClass.constBegin(); classIterator != markClass.constEnd(); ++classIterator)
    {
        classData.insert(QString::number(index), classIterator.key());
        index++;
    }
    jsonData.insert("DetectionClass", classData);
    doc.setObject(jsonData);
    data = doc.toJson();
    file.write(data);
    file.close();
    return 0;
}

int ManualParamterConfig::loadSegClassConfig(const QString &classPath)
{
    QByteArray data;
    QFile file;
    file.setFileName(classPath);
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
                if(jsonObject.contains("SegmentationClass"))
                {
                    QMap<QString, QVariant> readMarkClass = jsonObject.take("SegmentationClass").toObject().toVariantMap();
                    QString className;
                    QStringList color;
                    int red = 0;
                    int green = 0;
                    int blue = 0;
                    markClass.clear();
                    for(QMap<QString, QVariant>::const_iterator iter = readMarkClass.constBegin();
                        iter != readMarkClass.constEnd(); ++iter)
                    {
                        className = iter.key();
                        color = iter.value().toString().split(',');
                        red = color[0].toInt();
                        green = color[1].toInt();
                        blue = color[2].toInt();
                        QColor tempColor(red, green, blue);
                        markClass.insert(className, tempColor.name());
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

int ManualParamterConfig::saveSegClassConfig(const QString &classPath)
{
    if(markClass.count() == 0)
        return -2;
    QMap<QString, QString>::const_iterator classIterator;
    QJsonDocument doc;
    QByteArray data;
    QJsonObject jsonData;
    QJsonObject classData;
    QFile file(classPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate |QIODevice::Text))
    {
        return -1;
    }
    for(classIterator = markClass.constBegin(); classIterator != markClass.constEnd(); ++classIterator)
    {
        QColor tempColor(classIterator.value());
        classData.insert(classIterator.key(), QString("%1,%2,%3").arg(tempColor.red())
                        .arg(tempColor.green()).arg(tempColor.blue()));
    }
    jsonData.insert("SegmentationClass", classData);
    doc.setObject(jsonData);
    data = doc.toJson();
    file.write(data);
    file.close();
    return 0;
}

void ManualParamterConfig::init()
{

}
