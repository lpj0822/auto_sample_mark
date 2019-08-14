#ifndef XMLPROCESS_H
#define XMLPROCESS_H

#include <QObject>
#include <QString>
#include <QList>
#include <QDomComment>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>

#include "myobject.h"

class XMLProcess : public QObject
{
    Q_OBJECT
public:
    XMLProcess(QObject *parent = 0);
    ~XMLProcess();

    int createXML(const QString &xmlFilePath, const QString &imageFilePath, const int imageWidth,
                   const int inageHeight, const QList<MyObject> &objects);

    int readXML(const QString &xmlFilePath, QList<MyObject> &objects);

signals:

public slots:

private:

    int writeRectData(const MyObject &object, QXmlStreamWriter &xmlWriter);
    MyObject readRectData(const QDomNodeList &childList);

    int writeLineData(const MyObject &object, QXmlStreamWriter &xmlWriter);
    MyObject readLineData(const QDomNodeList &childList);

    int writePolygonData(const MyObject &object, QXmlStreamWriter &xmlWriter);
    MyObject readPolygonData(const QDomNodeList &childList);

    int writePointListData(const MyObject &object, QXmlStreamWriter &xmlWriter);
    MyObject readPointListData(const QDomNodeList &childList);
};

#endif // XMLPROCESS_H
