#pragma execution_character_set("utf-8")
#include "xmlprocess.h"
#include <QFileInfo>
#include <QDir>
#include <QMessageBox>
#include <QStringList>
#include <QDebug>
#include <iostream>

XMLProcess::XMLProcess(QObject *parent) : QObject(parent)
{

}

XMLProcess::~XMLProcess()
{

}

int XMLProcess::createXML(const QString &xmlFilePath, const QString &imageFilePath, const int imageWidth,
                          const int inageHeight, const QList<MyObject> &objects)
{
    int count = objects.size();
    QFileInfo fileInfo(imageFilePath);
    QFile file(xmlFilePath);
    QStringList pathNameList = fileInfo.absolutePath().split("/");
    QString folder = "MultipleTarget";
    if(pathNameList.size() > 0)
    {
        folder = pathNameList[pathNameList.size() - 1];
    }
    //qDebug() << "imageFilePath:" << fileInfo.absolutePath().split("/");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate |QIODevice::Text))
    {
        return -1;
    }

    QXmlStreamWriter xmlWriter(&file);
    xmlWriter.setAutoFormatting(true);
    xmlWriter.writeStartDocument();
    xmlWriter.writeStartElement("annotation");
    xmlWriter.writeTextElement("folder", folder);
    xmlWriter.writeTextElement("filename", fileInfo.fileName());
    xmlWriter.writeTextElement("path", imageFilePath);
    xmlWriter.writeStartElement("source");
    xmlWriter.writeTextElement("database", "lpj_dataset");
    xmlWriter.writeTextElement("annotation", "Annotations");
    xmlWriter.writeEndElement();
    xmlWriter.writeStartElement("owner");
    xmlWriter.writeTextElement("flickrid", "NULL");
    xmlWriter.writeTextElement("name", "lpj");
    xmlWriter.writeEndElement();
    xmlWriter.writeStartElement("size");
    xmlWriter.writeTextElement("width", QString("%1").arg(imageWidth));
    xmlWriter.writeTextElement("height", QString("%1").arg(inageHeight));
    xmlWriter.writeTextElement("depth", QString("%1").arg(3));
    xmlWriter.writeEndElement();
    for(int index = 0; index < count; index++)
    {
        const MyObject object = objects[index];
        if(object.getShapeType() == ShapeType::RECT_SHAPE)
        {
            writeRectData(object, xmlWriter);
        }
        else if(object.getShapeType() == ShapeType::LINE_SHAPE)
        {
            writeLineData(object, xmlWriter);
        }
        else if(object.getShapeType() == ShapeType::POLYGON_SHAPE)
        {
            writePolygonData(object, xmlWriter);
        }
        else if(object.getShapeType() == ShapeType::LANE_SEGMENT_SHAPE)
        {
            writePointListData(object, xmlWriter);
        }
    }
    xmlWriter.writeEndElement();
    xmlWriter.writeEndElement();
    xmlWriter.writeEndDocument();

    file.close();
    return 0;
}

int XMLProcess::readXML(const QString &xmlFilePath, QList<MyObject> &objects)
{
    QFile file(xmlFilePath);
    objects.clear();
    if(!file.open(QFile::ReadOnly | QFile::Text))
    {
        //QMessageBox::information(NULL, QString("XML"), QString("open XML error!"));
        return -1;
    }
    QDomDocument document;
    QString error;
    int row = 0;
    int column = 0;
    if(!document.setContent(&file, false, &error, &row, &column))
    {
        //QMessageBox::information(NULL, QString("XML"), QString("parse file failed at line row and column")
        //                         + QString::number(row, 10) + QString(",") + QString::number(column, 10));
        return -2;
    }

    if(document.isNull())
    {
        //QMessageBox::information(NULL, QString("XML"), QString("document is null!"));
        return -3;
    }
    QDomElement root = document.documentElement();
    QDomNodeList nodelist = root.childNodes();
    int nodeCount = nodelist.size();
    for(int index = 0; index < nodeCount; index++)
    {
        QDomNode domNode = nodelist.item(index);
        QDomElement element = domNode.toElement();
        if(element.tagName() == "rectObject")
        {
            QDomNodeList childList = element.childNodes();
            MyObject object = readRectData(childList);
            objects.append(object);
        }
        else if(element.tagName() == "lineObject")
        {
            QDomNodeList childList = element.childNodes();
            MyObject object = readLineData(childList);
            objects.append(object);
        }
        else if(element.tagName() == "polygonObject")
        {
            QDomNodeList childList = element.childNodes();
            MyObject object = readPolygonData(childList);
            objects.append(object);
        }
        else if(element.tagName() == "laneObject")
        {
            QDomNodeList childList = element.childNodes();
            MyObject object = readPointListData(childList);
            objects.append(object);
        }
    }
    return 0;
}

int XMLProcess::writeRectData(const MyObject &object, QXmlStreamWriter &xmlWriter)
{
    QRect rect = object.getBox();
    int difficult = static_cast<int>(object.getIsDifficult());
    if(rect.width() <= 0 || rect.height() <= 0)
    {
        qDebug() << "write rect data fail!\n" << endl;
        return -1;
    }
    xmlWriter.writeStartElement("rectObject");
    xmlWriter.writeTextElement("name", object.getObjectClass());
    xmlWriter.writeTextElement("pose", "Unspecified");
    xmlWriter.writeTextElement("truncated", QString("%1").arg(0));
    xmlWriter.writeTextElement("difficult", QString("%1").arg(difficult));
    xmlWriter.writeStartElement("bndbox");
    xmlWriter.writeTextElement("xmin", QString("%1").arg(rect.topLeft().x()));
    xmlWriter.writeTextElement("ymin", QString("%1").arg(rect.topLeft().y()));
    xmlWriter.writeTextElement("xmax", QString("%1").arg(rect.bottomRight().x()));
    xmlWriter.writeTextElement("ymax", QString("%1").arg(rect.bottomRight().y()));
    xmlWriter.writeEndElement();
    xmlWriter.writeEndElement();
    return 0;
}

MyObject XMLProcess::readRectData(const QDomNodeList &childList)
{
    MyObject object;
    for(int loop = 0; loop < childList.count(); loop++)
    {
        QDomNode childDomNode = childList.item(loop);
        QDomElement childElement = childDomNode.toElement();
        if(childElement.tagName() == "name")
        {
            object.setObjectClass(childElement.text());
        }
        else if(childElement.tagName() == "difficult")
        {
            int difficult = childElement.text().toInt();
            object.setIsDifficult(static_cast<bool>(difficult));
        }
        else if(childElement.tagName() == "bndbox")
        {
            QPoint topPoint;
            QPoint bottomPoint;
            QDomNodeList boxList = childElement.childNodes();
            for(int loop1 = 0; loop1 < boxList.size(); loop1++)
            {
                QDomNode boxDomNode = boxList.item(loop1);
                QDomElement boxElement = boxDomNode.toElement();
                if(boxElement.tagName() == "xmin")
                {
                    topPoint.setX(boxElement.text().toInt());
                }
                else if(boxElement.tagName() == "ymin")
                {
                    topPoint.setY(boxElement.text().toInt());
                }
                else if(boxElement.tagName() == "xmax")
                {
                    bottomPoint.setX(boxElement.text().toInt());
                }
                else if(boxElement.tagName() == "ymax")
                {
                    bottomPoint.setY(boxElement.text().toInt());
                }
            }
            object.setBox(QRect(topPoint, bottomPoint));
            object.setShapeType(ShapeType::RECT_SHAPE);
        }
    }
    return object;
}

int XMLProcess::writeLineData(const MyObject &object, QXmlStreamWriter &xmlWriter)
{
    QList<QPoint> line = object.getLine();
    int difficult = static_cast<int>(object.getIsDifficult());
    if(line.count() <= 0)
    {
        qDebug() << "write line data fail!\n" << endl;
        return -1;
    }
    xmlWriter.writeStartElement("lineObject");
    xmlWriter.writeTextElement("name", object.getObjectClass());
    xmlWriter.writeTextElement("pose", "Unspecified");
    xmlWriter.writeTextElement("truncated", QString("%1").arg(0));
    xmlWriter.writeTextElement("difficult", QString("%1").arg(difficult));
    xmlWriter.writeStartElement("line");
    xmlWriter.writeTextElement("x1", QString("%1").arg(line[0].x()));
    xmlWriter.writeTextElement("y1", QString("%1").arg(line[0].y()));
    xmlWriter.writeTextElement("x2", QString("%1").arg(line[1].x()));
    xmlWriter.writeTextElement("y2", QString("%1").arg(line[1].y()));
    xmlWriter.writeEndElement();
    xmlWriter.writeEndElement();
    return 0;
}

MyObject XMLProcess::readLineData(const QDomNodeList &childList)
{
    MyObject object;
    for(int loop = 0; loop < childList.count(); loop++)
    {
        QDomNode childDomNode = childList.item(loop);
        QDomElement childElement = childDomNode.toElement();
        if(childElement.tagName() == "name")
        {
            object.setObjectClass(childElement.text());
        }
        else if(childElement.tagName() == "difficult")
        {
            int difficult = childElement.text().toInt();
            object.setIsDifficult(static_cast<bool>(difficult));
        }
        else if(childElement.tagName() == "line")
        {
            QPoint point1;
            QPoint point2;
            QDomNodeList boxList = childElement.childNodes();
            for(int loop1 = 0; loop1 < boxList.size(); loop1++)
            {
                QDomNode boxDomNode = boxList.item(loop1);
                QDomElement boxElement = boxDomNode.toElement();
                if(boxElement.tagName() == "x1")
                {
                    point1.setX(boxElement.text().toInt());
                }
                else if(boxElement.tagName() == "y1")
                {
                    point1.setY(boxElement.text().toInt());
                }
                else if(boxElement.tagName() == "x2")
                {
                    point2.setX(boxElement.text().toInt());
                }
                else if(boxElement.tagName() == "y2")
                {
                    point2.setY(boxElement.text().toInt());
                }
            }
            object.setLine(point1, point2);
            object.setShapeType(ShapeType::LINE_SHAPE);
        }
    }
    return object;
}

int XMLProcess::writePolygonData(const MyObject &object, QXmlStreamWriter &xmlWriter)
{
    QPolygon polygon = object.getPolygon();
    int difficult = static_cast<int>(object.getIsDifficult());
    if(polygon.count() <= 0)
    {
        qDebug() << "write polygon data fail!\n" << endl;
        return -1;
    }
    xmlWriter.writeStartElement("polygonObject");
    xmlWriter.writeTextElement("name", object.getObjectClass());
    xmlWriter.writeTextElement("pose", "Unspecified");
    xmlWriter.writeTextElement("truncated", QString("%1").arg(0));
    xmlWriter.writeTextElement("difficult", QString("%1").arg(difficult));
    xmlWriter.writeTextElement("pointCount", QString("%1").arg(polygon.count()));
    xmlWriter.writeStartElement("polygon");
    for(int loop = 0; loop < polygon.count(); loop++)
    {
        xmlWriter.writeTextElement(QString("x%1").arg(loop), QString("%1").arg(polygon[loop].x()));
        xmlWriter.writeTextElement(QString("y%1").arg(loop), QString("%1").arg(polygon[loop].y()));
    }
    xmlWriter.writeEndElement();
    xmlWriter.writeEndElement();
    return 0;
}

MyObject XMLProcess::readPolygonData(const QDomNodeList &childList)
{
    MyObject object;
    for(int loop = 0; loop < childList.count(); loop++)
    {
        QDomNode childDomNode = childList.item(loop);
        QDomElement childElement = childDomNode.toElement();
        if(childElement.tagName() == "name")
        {
            object.setObjectClass(childElement.text());
        }
        else if(childElement.tagName() == "difficult")
        {
            int difficult = childElement.text().toInt();
            object.setIsDifficult(static_cast<bool>(difficult));
        }
        else if(childElement.tagName() == "polygon")
        {
            int index = 0;
            QPolygon polygon;
            QDomNodeList boxList = childElement.childNodes();
            polygon.clear();
            for(int loop1 = 0; loop1 < boxList.size(); loop1++)
            {
                QDomNode boxDomNode = boxList.item(loop1);
                QDomElement boxElement = boxDomNode.toElement();
                QPoint point;
                if(boxElement.tagName() == QString("x%1").arg(index))
                {
                    point.setX(boxElement.text().toInt());
                }
                loop1++;
                boxDomNode = boxList.item(loop1);
                boxElement = boxDomNode.toElement();
                if(boxElement.tagName() == QString("y%1").arg(index))
                {
                    point.setY(boxElement.text().toInt());
                }
                index++;
                polygon.append(point);
            }
            object.setPolygon(polygon);
            object.setShapeType(ShapeType::POLYGON_SHAPE);
        }
    }
    return object;
}

int XMLProcess::writePointListData(const MyObject &object, QXmlStreamWriter &xmlWriter)
{
    QList<QPoint> pointList = object.getPointList();
    int difficult = static_cast<int>(object.getIsDifficult());
    if(pointList.count() <= 0)
    {
        qDebug() << "write pointList data fail!\n" << endl;
        return -1;
    }
    xmlWriter.writeStartElement("laneObject");
    xmlWriter.writeTextElement("name", object.getObjectClass());
    xmlWriter.writeTextElement("pose", "Unspecified");
    xmlWriter.writeTextElement("truncated", QString("%1").arg(0));
    xmlWriter.writeTextElement("difficult", QString("%1").arg(difficult));
    xmlWriter.writeTextElement("pointCount", QString("%1").arg(pointList.count()));
    xmlWriter.writeStartElement("lane");
    for(int loop = 0; loop < pointList.count(); loop++)
    {
        xmlWriter.writeTextElement(QString("x%1").arg(loop), QString("%1").arg(pointList[loop].x()));
        xmlWriter.writeTextElement(QString("y%1").arg(loop), QString("%1").arg(pointList[loop].y()));
    }
    xmlWriter.writeEndElement();
    xmlWriter.writeEndElement();
    return 0;
}

MyObject XMLProcess::readPointListData(const QDomNodeList &childList)
{
    MyObject object;
    for(int loop = 0; loop < childList.count(); loop++)
    {
        QDomNode childDomNode = childList.item(loop);
        QDomElement childElement = childDomNode.toElement();
        if(childElement.tagName() == "name")
        {
            object.setObjectClass(childElement.text());
        }
        else if(childElement.tagName() == "difficult")
        {
            int difficult = childElement.text().toInt();
            object.setIsDifficult(static_cast<bool>(difficult));
        }
        else if(childElement.tagName() == "lane")
        {
            int index = 0;
            QList<QPoint> pointList;
            QDomNodeList boxList = childElement.childNodes();
            pointList.clear();
            for(int loop1 = 0; loop1 < boxList.size(); loop1++)
            {
                QDomNode boxDomNode = boxList.item(loop1);
                QDomElement boxElement = boxDomNode.toElement();
                QPoint point;
                if(boxElement.tagName() == QString("x%1").arg(index))
                {
                    point.setX(boxElement.text().toInt());
                }
                loop1++;
                boxDomNode = boxList.item(loop1);
                boxElement = boxDomNode.toElement();
                if(boxElement.tagName() == QString("y%1").arg(index))
                {
                    point.setY(boxElement.text().toInt());
                }
                index++;
                pointList.append(point);
            }
            object.setPointList(pointList);
            object.setShapeType(ShapeType::LANE_SEGMENT_SHAPE);
        }
    }
    return object;
}
