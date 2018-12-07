#include "videomultipletracking.h"
#include <QDir>
#include <iostream>
#include "multipletracking/kalmanmultipletracker.h"
#include "multipletracking/opencvmultipletracker.h"

VideoMultipletracking::VideoMultipletracking()
{
    init();
}

VideoMultipletracking::~VideoMultipletracking()
{
    if(multipleTracker != NULL)
    {
        delete multipleTracker;
        multipleTracker = NULL;
    }
}

void VideoMultipletracking::initTrackingData()
{
    hasObjects = false;
    if(multipleTracker != NULL)
    {
        multipleTracker->initTracking();
    }
}

void VideoMultipletracking::tracking(const cv::Mat preFrame, const cv::Mat &nextFrame,
                                     const QList<MyObject>& preObjects, const QList<MyObject>& nextObject)
{
    listObjects.clear();
    if(multipleTracker != NULL)
    {
        switch(method)
        {
        case TrackingMethod::KALMAN:
            {
                std::vector<TrackingObject> trackingObjects;
                QList<MyObject> updateObjects = updateRectTrackingObjects(preObjects, nextObject);
                leaveObjects(nextObject);
                if(!hasObjects)
                {
                    trackingObjects = getTrackingObjects(preObjects, ShapeType::RECT);
                    multipleTracker->mutilpleTracking(preFrame, nextFrame, trackingObjects);
                }
                trackingObjects = getTrackingObjects(updateObjects, ShapeType::RECT);
                multipleTracker->mutilpleTracking(preFrame, nextFrame, trackingObjects);
            }
            break;
        case TrackingMethod::KCF:
        case TrackingMethod::TLD:
        case TrackingMethod::MIL:
            {
                std::vector<TrackingObject> trackingObjects;
                QList<MyObject> updateObjects = updateRectTrackingObjects(preObjects);
                trackingObjects = getTrackingObjects(updateObjects, ShapeType::RECT);
                multipleTracker->mutilpleTracking(preFrame, nextFrame, trackingObjects);
            }
            break;
        }
    }
    else
    {
        for(int loop = 0; loop < nextObject.count(); loop++)
        {
            listObjects.append(nextObject[loop]);
        }
    }
}

QList<MyObject> VideoMultipletracking::getTrackingResult()
{
    QList<MyObject> result;
    result.clear();
    hasObjects = false;
    if(multipleTracker != NULL)
    {
        std::vector<TrackingObject> trackingObjects;
        multipleTracker->getTrackingResult(trackingObjects);
        int count = static_cast<int>(trackingObjects.size());
        if(count > 0)
        {
            hasObjects = true;
        }
        for(int loop = 0; loop < count; loop++)
        {
            cv::Rect rect = trackingObjects[loop].rect;
            MyObject object;
            object.setObjectClass(QString::fromStdString(trackingObjects[loop].objectClass));
            object.setBox(QRect(rect.x, rect.y, rect.width + 1, rect.height + 1));
            object.setShapeType(ShapeType::RECT);
            object.setIsTrackingObject(true);
            result.append(object);
        }
    }
    listObjects.append(result);
    return listObjects;
}

QList<MyObject> VideoMultipletracking::updateRectTrackingObjects(const QList<MyObject> &preObjects, const QList<MyObject> &nextObjects)
{
    QList<MyObject> result;
    int preCount = preObjects.count();
    int nextCount = nextObjects.count();
    result.clear();
    for(int preIndex = 0; preIndex < preCount; preIndex++)
    {
        if(preObjects[preIndex].getShapeType() == ShapeType::RECT && \
                !preObjects[preIndex].getIsTrackingObject())
        {
            QRect rect1 = preObjects[preIndex].getBox();
            int nextIndex = 0;
            for(; nextIndex < nextCount; nextIndex++)
            {
                if(nextObjects[nextIndex].getShapeType() == ShapeType::RECT)
                {
                    QRect rect2 = nextObjects[nextIndex].getBox();
                    if(geometryAlgorithm.rectOverlap(rect1, rect2) >= 0.45f)
                    {
                        break;
                    }
                }
            }
            if(nextIndex >= nextCount)
            {
                result.append(preObjects[preIndex]);
            }
        }
    }
    result.append(nextObjects);
    return result;
}

QList<MyObject> VideoMultipletracking::updateRectTrackingObjects(const QList<MyObject> &preObjects)
{
    QList<MyObject> result;
    int preCount = preObjects.count();
    result.clear();
    for(int preIndex = 0; preIndex < preCount; preIndex++)
    {
        if(preObjects[preIndex].getShapeType() == ShapeType::RECT && \
                !preObjects[preIndex].getIsTrackingObject())
        {
            result.append(preObjects[preIndex]);
        }
    }
    return result;
}

std::vector<TrackingObject> VideoMultipletracking::getTrackingObjects(const QList<MyObject>& objects, ShapeType type)
{
    std::vector<TrackingObject> trackingObjects;
    trackingObjects.clear();
    for(int loop = 0; loop < objects.count(); loop++)
    {
        if(objects[loop].getShapeType() == type)
        {
            QRect rect = objects[loop].getBox();
            int x = rect.topLeft().x();
            int y = rect.topLeft().y();
            int width = rect.bottomRight().x() - x;
            int height = rect.bottomRight().y() - y;
            TrackingObject trackingObject;
            trackingObject.rect = cv::Rect(x, y, width, height);
            trackingObject.objectClass = objects[loop].getObjectClass().toStdString();
            trackingObjects.push_back(trackingObject);
        }
    }
    return trackingObjects;
}

void VideoMultipletracking::leaveObjects(const QList<MyObject> &nextObjects)
{
    for(int loop = 0; loop < nextObjects.count(); loop++)
    {
        if(nextObjects[loop].getShapeType() != ShapeType::RECT)
        {
            listObjects.append(nextObjects[loop]);
        }
    }
}

void VideoMultipletracking::init()
{
    method = TrackingMethod::UNTRACKINGMETHOD;
    hasObjects = false;
    listObjects.clear();
    multipleTracker = NULL;
    QDir makeDir;
    if(!makeDir.exists("./config/"))
    {
        if(!makeDir.mkdir("./config/"))
        {
            std::cout<<"make dir fail!"<<std::endl;
            return;
        }
    }
    switch(VideoMarkParamterConfig::getTrackingMethod())
    {
    case TrackingMethod::KALMAN:
        method = TrackingMethod::KALMAN;
        multipleTracker = new KalmanMultipleTracker();
        break;
    case TrackingMethod::KCF:
        method = TrackingMethod::KCF;
        multipleTracker = new OpencvMultipletracker("KCF");
        break;
    case TrackingMethod::TLD:
        method = TrackingMethod::TLD;
        multipleTracker = new OpencvMultipletracker("TLD");
        break;
    case TrackingMethod::MIL:
        method = TrackingMethod::MIL;
        multipleTracker = new OpencvMultipletracker("MIL");
        break;
    default:
        break;
    }
}
