#pragma execution_character_set("utf-8")
#include "jsonprocess.h"
#include <QJsonParseError>
#include <QFile>
#include <QFileInfo>
#include <QVariant>
#include <QDebug>

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
    QStringList pathNameList = fileInfo.absolutePath().split("/");
    QString folder = "MultipleTarget";
    if(pathNameList.size() > 0)
    {
        folder = pathNameList[pathNameList.size() - 1];
    }
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate |QIODevice::Text))
    {
        return -1;
    }
    jsonData.insert("database", "lpj_dataset");
    jsonData.insert("annotation", "Annotations");
    jsonData.insert("folder", folder);
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
    writeLaneData(objects, allData);

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
                        QJsonArray rectValue = allData.take("rectObject").toArray();
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
                    if(allData.contains("laneObject"))
                    {
                        QJsonArray laneValue = allData.take("laneObject").toArray();
                        readLaneData(laneValue, objects);
                    }
                    if(allData.contains("rect3DObject"))
                    {
                        QJsonArray rect3DValue = allData.take("rect3DObject").toArray();
                        readRect3Ddata(rect3DValue, objects);
                    }
                }
            }
        }
    }
    else
    {
        qDebug() << "error:" << jsonError.errorString() << endl;
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
        if(objects[loop].getShapeType() == ShapeType::RECT_SHAPE)
        {
            const QRect rect = objects[loop].getBox();
            QJsonObject objectData;
            objectData.insert("minX", rect.topLeft().x());
            objectData.insert("minY", rect.topLeft().y());
            objectData.insert("maxX", rect.bottomRight().x());
            objectData.insert("maxY", rect.bottomRight().y());
            objectData.insert("class", objects[loop].getObjectClass());
            objectArrays.insert(index, objectData);
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
        QJsonObject objectData = value.at(loop).toObject();
        MyObject object;
        int minX = objectData.take("minX").toVariant().toInt();
        int minY = objectData.take("minY").toVariant().toInt();
        int maxX = objectData.take("maxX").toVariant().toInt();
        int maxY = objectData.take("maxY").toVariant().toInt();
        object.setObjectClass(objectData.take("class").toVariant().toString());
        object.setBox(QRect(QPoint(minX, minY), QPoint(maxX, maxY)));
        object.setShapeType(ShapeType::RECT_SHAPE);
        objects.append(object);
    }

    return 0;
}

int JSONProcess::writeLineData(const QList<MyObject>& objects, QJsonObject &jsonData)
{
    QJsonArray objectArrays;
    int index = 0;
    for(int loop = 0; loop < objects.count(); loop++)
    {
        if(objects[loop].getShapeType() == ShapeType::LINE_SHAPE)
        {
            const QList<QPoint> line = objects[loop].getLine();
            QJsonObject objectData;
            objectData.insert("x1", line[0].x());
            objectData.insert("y1", line[0].y());
            objectData.insert("x2", line[1].x());
            objectData.insert("y2", line[1].y());
            objectData.insert("class", objects[loop].getObjectClass());
            objectArrays.insert(index, objectData);
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
        QJsonObject objectData = value.at(loop).toObject();
        MyObject object;
        int x1 = objectData.take("x1").toVariant().toInt();
        int y1 = objectData.take("y1").toVariant().toInt();
        int x2 = objectData.take("x2").toVariant().toInt();
        int y2 = objectData.take("y2").toVariant().toInt();
        object.setObjectClass(objectData.take("class").toVariant().toString());
        object.setLine(QPoint(x1, y1), QPoint(x2, y2));
        object.setShapeType(ShapeType::LINE_SHAPE);
        objects.append(object);
    }

    return 0;
}

int JSONProcess::writePolygonData(const QList<MyObject>& objects, QJsonObject &jsonData)
{
    QJsonArray objectArrays;
    int dataIndex = 0;
    for(int loop = 0; loop < objects.size(); loop++)
    {
        if(objects[loop].getShapeType() == ShapeType::POLYGON_SHAPE)
        {
            const QPolygon polygon = objects[loop].getPolygon();
            QJsonObject objectData;
            QJsonArray polygonData;
            int tempIndex = 0;
            objectData.insert("class", objects[loop].getObjectClass());
            objectData.insert("pointCount", polygon.count());
            for(int index = 0; index < polygon.count(); index++)
            {
                polygonData.insert(tempIndex++, polygon[index].x());
                polygonData.insert(tempIndex++, polygon[index].y());
            }
            objectData.insert("polygon", polygonData);
            objectArrays.insert(dataIndex, objectData);
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
        QJsonObject objectData = value.at(loop).toObject();
        MyObject object;
        QPolygon polygon;
        QJsonArray dataList = objectData.take("polygon").toArray();
        polygon.clear();
        for(int index = 0; index < dataList.size(); index++)
        {
            QPoint point;
            point.setX(dataList.at(index).toInt());
            index++;
            point.setY(dataList.at(index).toInt());
            polygon.append(point);
        }
        object.setObjectClass(objectData.take("class").toVariant().toString());
        object.setPolygon(polygon);
        object.setShapeType(ShapeType::POLYGON_SHAPE);
        objects.append(object);
    }

    return 0;
}

int JSONProcess::writeLaneData(const QList<MyObject>& objects, QJsonObject &jsonData)
{
    QJsonArray objectArrays;
    int dataIndex = 0;
    for(int loop = 0; loop < objects.size(); loop++)
    {
        if(objects[loop].getShapeType() == ShapeType::LANE_SHAPE)
        {
            const QPolygon polygon = objects[loop].getPolygon();
            QJsonObject objectData;
            QJsonArray polygonData;
            int tempIndex = 0;
            objectData.insert("class", objects[loop].getObjectClass());
            objectData.insert("pointCount", polygon.count());
            for(int index = 0; index < polygon.count(); index++)
            {
                polygonData.insert(tempIndex++, polygon[index].x());
                polygonData.insert(tempIndex++, polygon[index].y());
            }
            objectData.insert("lane", polygonData);
            objectArrays.insert(dataIndex, objectData);
            dataIndex++;
        }
    }
    if(objectArrays.count() > 0)
    {
        jsonData.insert("laneObject", objectArrays);
    }
    return 0;
}

int JSONProcess::readLaneData(const QJsonArray &value, QList<MyObject>& objects)
{
    for(int loop = 0; loop < value.size(); loop++)
    {
        QJsonObject objectData = value.at(loop).toObject();
        MyObject object;
        QList<QPoint> pointList;
        QJsonArray dataList = objectData.take("lane").toArray();
        pointList.clear();
        for(int index = 0; index < dataList.size(); index++)
        {
            QPoint point;
            point.setX(dataList.at(index).toInt());
            index++;
            point.setY(dataList.at(index).toInt());
            pointList.append(point);
        }
        object.setObjectClass(objectData.take("class").toVariant().toString());
        object.setPointList(pointList);
        object.setShapeType(ShapeType::LANE_SHAPE);
        objects.append(object);
    }

    return 0;
}

int JSONProcess::readRect3Ddata(const QJsonArray &value, QList<MyObject>& objects)
{
    for(int loop = 0; loop < value.size(); loop++)
    {
        QJsonObject objectData = value.at(loop).toObject();
        MyObject object;
        MyRect3D rect3d;
        QString className;
        rect3d.center[0] = objectData.take("centerX").toVariant().toFloat();
        rect3d.center[1] = objectData.take("centerY").toVariant().toFloat();
        rect3d.center[2] = objectData.take("centerZ").toVariant().toFloat();
        rect3d.size[0] = objectData.take("width").toVariant().toFloat();
        rect3d.size[1] = objectData.take("length").toVariant().toFloat();
        rect3d.size[2] = objectData.take("height").toVariant().toFloat();
        rect3d.theta = objectData.take("yaw").toVariant().toFloat();
        className = objectData.take("class").toVariant().toString();
        object.setObjectClass(className);
        object.setBox3D(rect3d);
        object.setShapeType(ShapeType::RECT3D_SHAPE);
        objects.append(object);
    }
    return 0;
}
