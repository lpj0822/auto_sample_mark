#ifndef IMAGECONTROLWINDOW_H
#define IMAGECONTROLWINDOW_H

#include "controlwindow.h"
#include "helpers/videoprocess.h"
#include "videomultipletracking.h"

class ImageControlWindow : public ControlWindow
{
    Q_OBJECT

public:
    ImageControlWindow(QWidget *parent = 0);
    ~ImageControlWindow();

    void setMarkDataList(const QString markDataDir, const QList<QString> markDataList, const MarkDataType dataType);

public slots:

    void slotIsMark();
    void slotImageItem(QListWidgetItem *item);
    void slotChangeClass(QString classText);
    void slotShowFull();
    void slotScrollArea(int keyValue);

protected:
    void closeEvent(QCloseEvent *event);
    void keyPressEvent(QKeyEvent *e);

protected:

    void showPrevious();
    void showNext();

    void nextVideo();
    void previousVideo();

    void updateDrawLabel(bool isValue);
    void updateImage();
    void updateMarkProcessLable();

    void loadMarkData(const QString dataPath);
    void saveMarkDataResult();

    //imageData
    void loadImageData(const QString imagePath, const QString saveAnnotationsDir);
    void saveImageDataResult(const QString &saveAnnotationsDir, const QString &imagePath, const QList<MyObject> &objects);

    //videoData
    void loadVideoData(const QString videoPath, const QString saveAnnotationsDir);
    void saveVideoDataResult(const QString &saveAnnotationsDir, const QString &videoPath, const QList<MyObject> &objects);
    void loadVideoImage();
    void updateVideoResult(const QList<MyObject> &objects);
    void initVideoTracking();
    void videoTracking(const cv::Mat& preFrame, const cv::Mat& frame);

    void setMarkDataParamter();

    void initConnect();
    void initData();

    void initImageData();
    void initVideoData();

    void initMarkClassBox();
    void initImageList();

private:

    void loadMarkImage();
    void saveMarkImageResult();

private:

    cv::Mat preFrame;

    //imageData
    QString currentImagePath;

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

#endif // IMAGECONTROLWINDOW_H
