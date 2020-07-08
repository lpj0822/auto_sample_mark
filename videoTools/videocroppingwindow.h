#ifndef VIDEOCROPPINGWINDOW_H
#define VIDEOCROPPINGWINDOW_H

#include <QWidget>
#include <QSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QString>
#include <QCloseEvent>
#include <memory>
#include "helpers/videoprocess.h"
#include "croppingvideothread.h"

class VideoCroppingWindow : public QWidget
{
    Q_OBJECT
public:
    VideoCroppingWindow(QWidget *parent = 0);
    ~VideoCroppingWindow();

signals:
    void signalCloseVideoCroppingWindow(QString flag);

public slots:
    void slotOpen();
    void slotIsProcessAll(bool isChecked);
    void slotStart();
    void slotStop();
    void slotFinish(QString name);

protected:
    void closeEvent(QCloseEvent *event);

private:

    QVBoxLayout* mainLayout;

    QCheckBox* isAllCropBox;

    QLabel* scaleSizeLabel;
    QDoubleSpinBox *scaleSizeBox;//缩放尺寸输入
    QGroupBox *videoPosBox;
    QLabel* startLabel;
    QLabel* stopLabel;
    QSpinBox* startPosBox;
    QSpinBox* stopPosBox;

    QLineEdit *pathText;
    QPushButton* openButton;
    QPushButton* startButton;
    QPushButton* stopButton;

    QLabel* infoLabel;

    std::shared_ptr<VideoProcess> videoProcess;
    CroppingVideoThread *croppingVideo;

    QString path;

private:

    void init();
    void initUI();
    void initConnect();
    void closeCurrentVideo();
};

#endif // VIDEOCROPPINGWINDOW_H
