#ifndef FROMVIDEOTOPICTUREWINDOW_H
#define FROMVIDEOTOPICTUREWINDOW_H

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
#include "saveData/saveimagethread.h"
#include "helpers/videoprocess.h"
#include "helpers/recordhistorydata.h"
#include "utilityGUI/customWindow/mytextbrowser.h"

class FromVideoToPictureWindow : public QWidget
{
    Q_OBJECT
public:
    FromVideoToPictureWindow(QWidget *parent = 0);
    ~FromVideoToPictureWindow();

signals:
    void signalCloseVideoToPictureWindow(QString flag);

public slots:
    void slotOpenDir();
    void slotStart();
    void slotStop();
    void slotFinish(QString name);

     void slotIsProcessDir(bool isChecked);
     void slotVideoCurrentFrame(int number);

protected:
    void closeEvent(QCloseEvent *event);
    void contextMenuEvent (QContextMenuEvent * event);

private:
    void init();
    void initUI();
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

    QLabel* skipFrameLabel;
    QSpinBox* skipFrameBox;
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
    SaveImageThread* saveImage;

    RecordHistoryData historyProcess;

    QList<QString> processVideoList;
    QList<QString> historyVideo;
    QString videoPathDir;

    QString currentProcessVideo;
    int currentFrameNumber;

    bool isStopAll;

};

#endif // FROMVIDEOTOPICTUREWINDOW_H
