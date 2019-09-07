#include "segmentimagesave.h"

SegmentImageSave::SegmentImageSave(QObject *parent) : QObject(parent)
{

}

SegmentImageSave::~SegmentImageSave()
{

}

int SegmentImageSave::createSegmentImage()
{
    return 0;
}

int SegmentImageSave::readSegmentImage(const QString &imageFilePath, MyObject &object)
{
    QImage segmentImage;
    if(segmentImage.load(imageFilePath))
    {
        object.setSegmentImage(segmentImage);
        return 0;
    }
    else
    {
        return -1;
    }
}
