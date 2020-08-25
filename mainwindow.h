#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QAction>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QMenuBar>
#include <QMenu>
#include <QToolBar>
#include <QAction>
#include <QComboBox>
#include <QStackedWidget>
#include "videoTools/segmentationlabelconvertwindow.h"
#include "videoTools/fromvideotopicturewindow.h"
#include "videoTools/frompicturetovideowindow.h"
#include "videoTools/videocuttingwindow.h"
#include "videoTools/videocroppingwindow.h"
#include "videoTools/imageconverterwindow.h"
#include "videoTools/qcamerawindow.h"
#include "pcTools/pcdconverterwindow.h"
#include "pcTools/pcdfilterwindow.h"
#include "autoSampleMark/autosamplemarkwindow.h"
#include "drawShape/myshape.h"
#include "controlwindow.h"
#include "imagecontrolwindow.h"
#include "videocontrolwindow.h"
#include "imagesegmentcontrolwindow.h"
#include "pclcontrolwindow.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

signals:
    void signalManualMarkParamterChanged();

public slots:
    //file
    void slotOpenImageDir();
    void slotOpenVideoDir();
    void slotOpenImageSegmentDir();
    void slotOpenPCDDir();
    //setting
    void slotManualMarkParamterConfig();
    void slotSegmentMarkParamterConfig();
    void slotAutoMarkParamterConfig();
    void slotVideoMarkParamterConfig();

    void slotPointCloudParamterConfig();
    //autoMark
    void slotAutoSampleMark();
    //tool
    void slotSegLabelConvert();

    void slotVideoToPicture();
    void slotVideoFromPicture();
    void slotVideoCropping();
    void slotVideoCutting();
    void slotImageConverter();
    void slotCamera();

    void slotPcdConverter();
    void slotPcdFilter();

    //about
    void slotAbout();
    void slotUserManual();

    //shapeTool
    void slotSelectMarkShape(const QString &text);

    void slotCloseOtherWindow(QString flag);

protected:
    void closeEvent(QCloseEvent *event);

private:

    //Action
    //file
    QAction *openImageDirAction;
    QAction *openVideoDirAction;
    QAction *openSegmentImageDirAction;
    QAction *openPCDDirAction;
    QAction *exitAction;
    //setting
    QAction *manualParamterAction;
    QAction *segmentParamterAction;
    QAction *autoParamterAction;
    QAction *videoMarkParamterAction;
    QAction *pointcloudParamterAction;
    //autoMark
    QAction *autoMarkAction;
    //tool
    QAction *segLabelConvertAction;
    QAction *videoToPictureAction;
    QAction *videoFromPictureAction;
    QAction *videoCuttingAction;
    QAction *videoCroppingAction;
    QAction *imageConverterAction;
    QAction *cameraAction;
    QAction *pcdConverterAction;
    QAction *pcdFilterAction;
    //about
    QAction *aboutAction;
    QAction *userManualAction;

    //Menu
    QMenu *fileMenu;
    QMenu *settingMenu;
    QMenu *autoMarkMenu;
    QMenu *toolMenu;
    QMenu *aboutMenu;

    //ToolBar
    QToolBar *fileTool;
    QToolBar *autoMarkTool;
    QToolBar *shapeTool;

    //shapeTool
    QWidget *shapeWidget;
    QLabel *shapeLabel;
    QComboBox *shapeBox;

    QStackedWidget *centerWidget;
    QList<ControlWindow *> markWindow;

private:
    AutoSampleMarkWindow *autoSampleMarkWindow;

    SegmentationLabelConvertWindow *segLabelConvertWindow;
    FromVideoToPictureWindow *videoToPictureWindow;
    FromPictureToVideoWindow *videoFromPictureWindow;
    VideoCuttingWindow *videoCuttingWindow;
    VideoCroppingWindow *videoCroppingWindow;
    ImageConverterWindow *imageConverterWindow;
    QCameraWindow *cameraWindow;

    PCDConverterWindow *pcdConverterWindow;
    PCDFilterWindow *pcdFilterWindow;

    MarkDataType loadDataType;
    MyShape imgShape;

    QString openDataDir;

private:
    void initData();
    void initAction();
    void initMenuBar();
    void initToolBar();
    void initUI();
    void initConnect();

    void initImageMarkShape();
    void initSegmentMarkShape();
    void initPointCloudMarkShape();

};

#endif // MAINWINDOW_H
