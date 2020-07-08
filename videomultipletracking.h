#ifndef VIDEOMULTIPLETRACKING_H
#define VIDEOMULTIPLETRACKING_H

#include "dataType/myobject.h"
#include "multipletracking/imultipletracker.h"
#include "baseAlgorithm/geometryalgorithm.h"
#include "sampleMarkParam/videomarkparamterconfig.h"

class VideoMultipletracking
{
public:
    VideoMultipletracking();
    ~VideoMultipletracking();

    void initTrackingData();
    void tracking(const cv::Mat preFrame, const cv::Mat &nextFrame,
                  const QList<MyObject>& preObjects, const QList<MyObject>& nextObject);
    QList<MyObject> getTrackingResult();

private:

    TrackingMethod method;
    IMultipletracker *multipleTracker;
    QList<MyObject> listObjects;
    bool hasObjects;
    GeometryAlgorithm geometryAlgorithm;

private:

    QList<MyObject> updateRectTrackingObjects(const QList<MyObject> &preObjects, const QList<MyObject> &nextObjects);
    QList<MyObject> updateRectTrackingObjects(const QList<MyObject> &preObjects);
    std::vector<TrackingObject> getTrackingObjects(const QList<MyObject>& objects, ShapeType type);

    void leaveObjects(const QList<MyObject> &nextObjects);

    void init();
};

#endif // VIDEOMULTIPLETRACKING_H
