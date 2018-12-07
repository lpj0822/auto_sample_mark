#ifndef JSONPROCESS_H
#define JSONPROCESS_H

#include <QObject>
#include <QMap>
#include <QList>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include "myobject.h"

class JSONProcess : public QObject
{
    Q_OBJECT
public:
    JSONProcess(QObject *parent = 0);
    ~JSONProcess();

    int createJSON(const QString &jsonFilePath, const QString &imageFilePath, const int imageWidth,
                   const int inageHeight, const QList<MyObject> &objects);

    int readJSON(const QString &jsonFilePath, QList<MyObject> &objects);

signals:

public slots:

protected:

    int writeRectData(const QList<MyObject>& objects, QJsonObject &jsonData);
    int readRectData(const QJsonArray &value, QList<MyObject>& objects);

    int writeLineData(const QList<MyObject>& objects, QJsonObject &jsonData);
    int readLineData(const QJsonArray &value, QList<MyObject>& objects);

    int writePolygonData(const QList<MyObject>& objects, QJsonObject &jsonData);
    int readPolygonData(const QJsonArray &value, QList<MyObject>& objects);

};

#endif // JSONPROCESS_H
