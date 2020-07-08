#ifndef SEGMENTIMAGESAVE_H
#define SEGMENTIMAGESAVE_H

#include <QObject>
#include <QImage>
#include <QColor>
#include "dataType/myobject.h"

class SegmentImageSave : public QObject
{
    Q_OBJECT
public:
    SegmentImageSave(QObject *parent = nullptr);
    ~SegmentImageSave();

    int createSegmentImage();
    int readSegmentImage(const QString &imageFilePath, MyObject &object);

    QImage generateMaskFromPolygon(const QList<MyObject> &obejcts, const int width, const int height);

signals:

public slots:

};

#endif // SEGMENTIMAGESAVE_H
