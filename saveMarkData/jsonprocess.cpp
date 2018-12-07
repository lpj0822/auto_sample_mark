﻿#pragma execution_character_set("utf-8")
#include "jsonprocess.h"
#include <QJsonParseError>
#include <QFile>
#include <QFileInfo>
#include <QVariant>

JSONProcess::JSONProcess(QObject *parent) : QObject(parent)
{

}

JSONProcess::~JSONProcess()
{

}

int JSONProcess::createJSON(const QString &jsonFilePath, const QString &imageFilePath, const int imageWidth,
               const int inageHeight, const QList<MyObject> &objects)
{
    const int count = objects.size();
    QJsonDocument doc;
    QByteArray data;
    QJsonObject jsonData;
    QJsonObject sizeData;
    QJsonObject allData;
    QFileInfo fileInfo(imageFilePath);
    QFile file(jsonFilePath);

    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate |QIODevice::Text))
    {
        return -1;
    }
    jsonData.insert("database", "lpj_dataset");
    jsonData.insert("annotation", "Annotations");
    jsonData.insert("folder", "MultipleTarget");
    jsonData.insert("filename", fileInfo.fileName());
    jsonData.insert("path", imageFilePath);
    jsonData.insert("owner", "lpj");

    //size
    sizeData.insert("width", imageWidth);
    sizeData.insert("height", inageHeight);
    jsonData.insert("size", sizeData);

    jsonData.insert("objectCount", count);

    writeRectData(objects, allData);
    writeLineData(objects, allData);
    writePolygonData(objects, allData);

    jsonData.insert("objects", allData);
    doc.setObject(jsonData);
    data = doc.toJson();
    file.write(data);
    file.close();
    return 0;
}

int JSONProcess::readJSON(const QString &jsonFilePath, QList<MyObject> &objects)
{
    QByteArray data;
    QFile file;
    file.setFileName(jsonFilePath);
    if(!file.open(QFile::ReadOnly | QFile::Text))
    {
        return -1;
    }
    data = file.readAll();
    file.close();
    QJsonParseError jsonError;
    QJsonDocument parseDoucment = QJsonDocument::fromJson(QString(data).toUtf8(), &jsonError);

    objects.clear();

    if(jsonError.error == QJsonParseError::NoError)
    {
        if (!(parseDoucment.isNull() || parseDoucment.isEmpty()))
        {
            if (parseDoucment.isObject())
            {
                QJsonObject jsonObject = parseDoucment.object();
                if(jsonObject.contains("objects"))
                {
                    QJsonObject allData = jsonObject.take("objects").toObject();
                    if(allData.contains("rectObject"))
                    {
                        QJsonArray rectValue = allData.take("skipNumber").toArray();
                        readRectData(rectValue, objects);
                    }
                    if(allData.contains("lineObject"))
                    {
                        QJsonArray lineValue = allData.take("lineObject").toArray();
                        readLineData(lineValue, objects);
                    }
                    if(allData.contains("polygonObject"))
                    {
                        QJsonArray polygonValue = allData.take("polygonObject").toArray();
                        readPolygonData(polygonValue, objects);
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

int JSONProcess::writeRectData(const QList<MyObject>& objects, QJsonObject &jsonData)
{
    QJsonArray objectArrays;
    int index = 0;
    for(int loop = 0; loop < objects.size(); loop++)
    {
        if(objects[loop].getShapeType() == ShapeType::RECT)
        {
            const QRect rect = objects[loop].getBox();
            objectArrays.insert(index, QString::fromLocal8Bit("%1 %2 %3 %4 %5").arg(objects[loop].getObjectClass())
                                .arg(rect.topLeft().x())
                                .arg(rect.topLeft().y())
                                .arg(rect.bottomRight().x())
                                .arg(rect.bottomRight().y()));
            index++;
        }
    }
    if(objectArrays.count() > 0)
    {
        jsonData.insert("rectObject", objectArrays);
    }
    return 0;
}

int JSONProcess::readRectData(const QJsonArray &value, QList<MyObject>& objects)
{
    for(int loop = 0; loop < value.size(); loop++)
    {
        QString objectData = value.at(loop).toString();
        MyObject object;
        QStringList dataList = objectData.split(" ");
        if(dataList.size() == 5)
        {
            object.setObjectClass(dataList[0]);
            object.setBox(QRect(QPoint(dataList[1].toInt(), dataList[2].toInt()),
                    QPoint(dataList[3].toInt(), dataList[4].toInt())));
            object.setShapeType(ShapeType::RECT);
            objects.append(object);
        }
    }

    return 0;
}

int JSONProcess::writeLineData(const QList<MyObject>& objects, QJsonObject &jsonData)
{
    QJsonArray objectArrays;
    int index = 0;
    for(int loop = 0; loop < objects.count(); loop++)
    {
        if(objects[loop].getShapeType() == ShapeType::LINE)
        {
            const QList<QPoint> line = objects[loop].getLine();
            objectArrays.insert(index, QString::fromLocal8Bit("%1 %2 %3 %4 %5").arg(objects[loop].getObjectClass())
                                .arg(line[0].x())
                                .arg(line[0].y())
                                .arg(line[1].x())
                                .arg(line[1].y()));
            index++;
        }
    }
    if(objectArrays.count() > 0)
    {
        jsonData.insert("lineObject", objectArrays);
    }
    return 0;
}

int JSONProcess::readLineData(const QJsonArray &value, QList<MyObject>& objects)
{
    for(int loop = 0; loop < value.size(); loop++)
    {
        QString objectData = value.at(loop).toString();
        MyObject object;
        QStringList dataList = objectData.split(" ");
        if(dataList.size() == 5)
        {
            object.setObjectClass(dataList[0]);
            object.setLine(QPoint(dataList[1].toInt(), dataList[2].toInt()),
                    QPoint(dataList[3].toInt(), dataList[4].toInt()));
            object.setShapeType(ShapeType::LINE);
            objects.append(object);
        }
    }

    return 0;
}

int JSONProcess::writePolygonData(const QList<MyObject>& objects, QJsonObject &jsonData)
{
    QJsonArray objectArrays;
    int dataIndex = 0;
    for(int loop = 0; loop < objects.size(); loop++)
    {
        if(objects[loop].getShapeType() == ShapeType::POLYGON)
        {
            const QPolygon polygon = objects[loop].getPolygon();
            QString writeData = QString::fromLocal8Bit("%1").arg(objects[loop].getObjectClass());
            for(int index = 0; index < polygon.count(); index++)
            {
                writeData += QString::fromLocal8Bit(" %1 %2").arg(polygon[index].x()).arg(polygon[index].y());
            }
            objectArrays.insert(dataIndex, writeData);
            dataIndex++;
        }
    }
    if(objectArrays.count() > 0)
    {
        jsonData.insert("polygonObject", objectArrays);
    }
    return 0;
}

int JSONProcess::readPolygonData(const QJsonArray &value, QList<MyObject>& objects)
{
    for(int loop = 0; loop < value.size(); loop++)
    {
        QString objectData = value.at(loop).toString();
        MyObject object;
        QPolygon polygon;
        QStringList dataList = objectData.split(" ");
        polygon.clear();
        for(int index = 0; index < dataList.count(); index++)
        {
            if(index == 0)
            {
                object.setObjectClass(dataList[index]);
            }
            else
            {
                QPoint point;
                point.setX(dataList[index].toInt());
                index++;
                point.setY(dataList[index].toInt());
                polygon.append(point);
            }
        }
        object.setPolygon(polygon);
        object.setShapeType(ShapeType::POLYGON);
        objects.append(object);
    }

    return 0;
}