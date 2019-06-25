#ifndef VIDEOCONTROLWINDOW_H
#define VIDEOCONTROLWINDOW_H

#include "imagecontrolwindow.h"
#include "helpers/videoprocess.h"
#include "videomultipletracking.h"

class VideoControlWindow : public ImageControlWindow
{
    Q_OBJECT

public:
    VideoControlWindow(QWidget *parent = 0);
    ~VideoControlWindow();

    void setMarkDataList(const QString markDataDir, const QList<QString> markDataList, const MarkDataType dataType);

public slots:

    void slotScrollArea(int keyValue);

protected:
    void keyPressEvent(QKeyEvent *e);

private:
    void showPrevious();
    void showNext();

    void nextVideo();
    void previousVideo();

    void updateLabelText(int markCount);

    void loadMarkData(const QString dataPath);
    void saveMarkDataResult();
    void loadMarkImage();
    void saveMarkImageResult();

    //videoData
    void loadVideoData(const QString videoPath, const QString saveAnnotationsDir);
    void saveVideoDataResult(const QString &saveAnnotationsDir, const QString &videoPath, const QList<MyObject> &objects);
    void loadVideoImage();
    void updateVideoResult(const QList<MyObject> &objects);
    void initVideoTracking();
    void videoTracking(const cv::Mat& preFrame, const cv::Mat& frame);

    void setVideoMarkDataParamter();

    void initData();
    void initVideoData();

private:

    cv::Mat preFrame;
    //videoData
    QString currentVideoPath;
    int currentFrameNumber;
    int allCountFrame;
    int skipFrameNumber;
    bool videoIsTracking;
    QMap<int, QList<MyObject> > videoResult;

    VideoMultipletracking *videoMultipletracking;
    VideoProcess videoProcess;
};

#endif // VIDEOCONTROLWINDOW_H
