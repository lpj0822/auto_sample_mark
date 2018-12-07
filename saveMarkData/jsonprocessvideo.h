#ifndef JSONPROCESSVIDEO_H
#define JSONPROCESSVIDEO_H

#include "jsonprocess.h"

class JSONProcessVideo : public JSONProcess
{
    Q_OBJECT
public:
    JSONProcessVideo(QObject *parent = 0);
    ~JSONProcessVideo();

    int createJSON(const QString &jsonFilePath, const QString &videoFilePath, const QMap<int, QList<MyObject> > &result,
                   const int skipNumber);

    int readJSON(const QString &jsonFilePath, QMap<int, QList<MyObject> > &result, int &skipNumber);

signals:

public slots:

};

#endif // JSONPROCESSVIDEO_H
