#ifndef SEGMENTIMAGESAVE_H
#define SEGMENTIMAGESAVE_H

#include <QObject>
#include "dataType/myobject.h"

class SegmentImageSave : public QObject
{
    Q_OBJECT
public:
    SegmentImageSave(QObject *parent = nullptr);
    ~SegmentImageSave();

    int createSegmentImage();
    int readSegmentImage(const QString &imageFilePath, MyObject &object);

signals:

public slots:

};

#endif // SEGMENTIMAGESAVE_H
