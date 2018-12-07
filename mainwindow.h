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
#include "videoTools/fromvideotopicturewindow.h"
#include "videoTools/frompicturetovideowindow.h"
#include "videoTools/videocuttingwindow.h"
#include "videoTools/videocroppingwindow.h"
#include "videoTools/qcamerawindow.h"
#include "autoSampleMark/autosamplemarkwindow.h"
#include "controlwindow.h"
#include "drawShape/myshape.h"

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
    //setting
    void slotManualMarkParamterConfig();
    void slotAutoMarkParamterConfig();
    void slotVideoMarkParamterConfig();
    //autoMark
    void slotAutoSampleMark();
    //tool
    void slotVideoToPicture();
    void slotVideoFromPicture();
    void slotVideoCropping();
    void slotVideoCutting();
    void slotCamera();
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
    QAction *exitAction;
    //setting
    QAction *manualParamterAction;
    QAction *autoParamterAction;
    QAction *videoMarkParamterAction;
    //autoMark
    QAction *autoMarkAction;
    //tool
    QAction *videoToPictureAction;
    QAction *videoFromPictureAction;
    QAction *videoCuttingAction;
    QAction *videoCroppingAction;
    QAction *cameraAction;
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

    ControlWindow *centerWidget;

private:
    AutoSampleMarkWindow *autoSampleMarkWindow;

    FromVideoToPictureWindow *videoToPictureWindow;
    FromPictureToVideoWindow *videoFromPictureWindow;
    VideoCuttingWindow *videoCuttingWindow;
    VideoCroppingWindow *videoCroppingWindow;
    QCameraWindow *cameraWindow;

    MyShape myShape;

    QString openDataDir;

private:
    void initData();
    void initAction();
    void initMenuBar();
    void initToolBar();
    void initUI();
    void initConnect();

};

#endif // MAINWINDOW_H
