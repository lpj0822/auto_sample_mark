#ifndef VIDEOCUTTINGWINDOW_H
#define VIDEOCUTTINGWINDOW_H

#include <QWidget>
#include <QSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QListWidget>
#include <QGroupBox>
#include <QProgressBar>
#include <QAction>
#include <QString>
#include <memory>
#include "helpers/videoprocess.h"
#include "helpers/recordhistorydata.h"
#include "utilityGUI/customWindow/mytextbrowser.h"
#include "croppingvideothread.h"

class VideoCuttingWindow : public QWidget
{
    Q_OBJECT
public:
    VideoCuttingWindow(QWidget *parent = 0);
    ~VideoCuttingWindow();

signals:
    void signalCloseVideoCuttingWindow(QString flag);

public slots:
    void slotOpenDir();
    void slotStart();
    void slotStop();
    void slotFinish(QString name);

    void slotIsProcessDir(bool isChecked);

protected:
    void closeEvent(QCloseEvent *event);
    void contextMenuEvent (QContextMenuEvent * event);

private:
    void init();
    void initUI();
    void initMenu();
    void initConnect();

    void initProcessVideoList(const QList<QString>& pathList);
    void updateProcessVideo();
    void updateListBox();
    void updateProgressBar();
    void startCutVideo();
    void nextCutVideo();
    void closeCurrentVideo();

private:

    QGroupBox *centerGroundBox;

    QCheckBox *isProcessDirBox;

    QLabel *videoPostLabel;
    QComboBox *videoPostBox;

    QLabel* skipTimeLabel;
    QSpinBox* skipTimeBox;
    QLabel* postLabel;
    QComboBox* postBox;

    QListWidget *videoListWidget;

    QLineEdit *pathText;
    QPushButton* openButton;
    QPushButton* startProcessButton;
    QPushButton* stopProcessButton;

    QLabel *videoProgressLabel;
    QProgressBar *videoProgressBar;

    MyTextBrowser *commandText;//输出黑匣子指令

    //menu
    QAction *startVideoProcessAction;
    QAction *stopVideoProcessAction;

 private:

    std::shared_ptr<VideoProcess> videoProcess;
    CroppingVideoThread *croppingVideo;

    RecordHistoryData historyProcess;

    QList<QString> processVideoList;
    QList<QString> historyVideo;
    QString videoPathDir;

    QString currentProcessVideo;
    QString currentVideoSaveDir;
    QString currentVideoName;
    int cutNumber;
    int videoStartPos;
    int videoStopPos;
    int countMesc;

    bool isStopAll;
};

#endif // VIDEOCUTTINGWINDOW_H
