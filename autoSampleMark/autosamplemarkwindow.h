#ifndef AUTOSAMPLEMARKWINDOW_H
#define AUTOSAMPLEMARKWINDOW_H

#include <QWidget>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QProgressBar>
#include <QListWidget>
#include <QGroupBox>
#include <QAction>

#include "utilityGUI/customWindow/mytextbrowser.h"
#include "helpers/recordhistorydata.h"
#include "autosamplemarkthread.h"



class AutoSampleMarkWindow : public QWidget
{
    Q_OBJECT

public:
    AutoSampleMarkWindow(QWidget *parent = 0);
    ~AutoSampleMarkWindow();

    void initData();

signals:
    void signalCloseAutoSampleMarkWindow(QString flag);

public slots:
    void slotOpenDir();
    void slotStart();
    void slotStop();
    void slotIsProcessDir(bool isChecked);
    void slotVideoInit(int allFrameCount);
    void slotVideoCurrentFrame(int number);
    void slotVideoFinish(QString videoInfo);
    void slotFinishAll();

protected:

    void closeEvent(QCloseEvent *event);
    void contextMenuEvent (QContextMenuEvent * event);

private:

    void initMainUI();
    void initMenu();
    void initConnect();

    void initProcessVideoList(const QList<QString>& pathList);
    void updateListBox();
    void updateProgressBar();

private:

    QGroupBox *centerGroundBox;

    QCheckBox *isProcessDirBox;

    QLabel *videoPostLabel;
    QComboBox *videoPostBox;

    QLabel *skipFrameLabel;
    QSpinBox *skipFrameBox;

    QLabel *saveImagePostLabel;
    QComboBox *saveImagePostBox;

    QLabel *confidenceThresholdLabel;
    QDoubleSpinBox *confidenceThresholdBox;

    QLabel *videoDirLabel;
    QLineEdit *videoDirText;
    QPushButton *openVideoDirButton;

    QListWidget *videoListWidget;

    QPushButton *startProcessButton;
    QPushButton *stopProcessButton;

    QLabel *videoProgressLabel;
    QProgressBar *videoProgressBar;

    MyTextBrowser *commandText;//输出黑匣子指令

    //menu
    QAction *startVideoProcessAction;
    QAction *stopVideoProcessAction;

private:

    QList<QString> processVideoList;
    QList<QString> historyVideo;
    QString videoPathDir;

    RecordHistoryData historyProcess;
    AutoSampleMarkThread sampleMarkThread;

    QString currentProcessVideo;

    int currentFrameNumber;
};


#endif // AUTOSAMPLEMARKWINDOW_H
