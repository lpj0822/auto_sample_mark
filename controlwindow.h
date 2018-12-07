#ifndef CONTROLWINDOW_H
#define CONTROLWINDOW_H

#include <QWidget>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QTextBrowser>
#include <QCheckBox>
#include <QProgressBar>
#include <QListWidget>
#include <QGroupBox>
#include <QAction>
#include <QScrollArea>
#include <QToolButton>
#include <QMap>

#include "helpers/videoprocess.h"
#include "helpers/recordhistorydata.h"
#include "saveMarkData/xmlprocess.h"
#include "saveMarkData/jsonprocessvideo.h"
#include "utilityGUI/customWindow/wexpand.h"
#include "utilityGUI/customWindow/customanimation.h"
#include "utilityGUI/customWindow/myscrollarea.h"
#include "videomultipletracking.h"
#include "editablelabel.h"

typedef enum MarkDataType{
    UNKNOWN = -1,
    IMAGE = 0,
    VIDEO = 1
}MarkDataType;


class ControlWindow : public QWidget
{
    Q_OBJECT

public:
    ControlWindow(QWidget *parent = 0);
    ~ControlWindow();

    void setMarkDataList(const QString markDataDir, const QList<QString> markDataList, const MarkDataType dataType);

    void setDrawShape(int shapeId);

public slots:

    void slotIsMark();
    void slotImageItem(QListWidgetItem *item);
    void slotChangeClass(QString classText);
    void slotShowFull();
    void slotScrollArea(int keyValue);

    void slotManualMarkParamterChanged();

protected:

    void closeEvent(QCloseEvent *event);
    void resizeEvent(QResizeEvent *e);
    void contextMenuEvent (QContextMenuEvent * event);
    void keyPressEvent(QKeyEvent *e);

private:

    void showPrevious();
    void showNext();

    void nextVideo();
    void previousVideo();

    void updateDrawLabel(bool isValue);
    void updateIsMarkButton(bool isValue);
    void updateImage();
    void updateListBox();
    void updateMarkProcessLable();

    void loadMarkData(const QString dataPath);
    void saveMarkDataResult();

    void loadMarkImage();
    void saveMarkImageResult();

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

    void initUI();
    void initConnect();
    void initData();

    void initMarkData(const QString dirPath, const MarkDataType dataType);
    void initImageData();
    void initVideoData();

    void initMarkClassBox();
    void initImageList();

    void initExpandLeft();
    void initExpandRight();
    void updateExpandLeft();
    void updateExpandRight();

    void readMarkHistory();
    void writeMarkHistory();

private: 

    QGroupBox *centerTopBox;

    QWidget *centerWidget;
    QLabel *showClass;
    QComboBox *classBox;
    QPushButton *showFullButton;
    QPushButton *isMarkButton;
    QLabel *markProcessLabel;

    EditableLabel *drawLable;
    MyScrollArea *drawScrollArea;
    QListWidget *imageListWidget;

    WExpand *expand1;
    CustomAnimation *customAnimation1;
    bool leftTabXxpanded1;
    int leftTabminmumwidth1;
    WExpand *expand2;
    CustomAnimation *customAnimation2;
    bool leftTabXxpanded2;
    int leftTabminmumwidth2;

private:

    QList<QString> processMarkDataList;
    QList<int> processDataFlagList;
    QString markDataDir;

    bool isMark;
    MarkDataType markDataType;

    cv::Mat preFrame;
    QImage currentImage;
    int currentIndex;

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

    JSONProcessVideo jsonProcess;
    XMLProcess xmlProcess;
    VideoProcess videoProcess;
    RecordHistoryData historyProcess;
};

#endif // CONTROLWINDOW_H
