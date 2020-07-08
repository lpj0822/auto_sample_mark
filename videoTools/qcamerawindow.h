#ifndef QCAMERAWINDOW_H
#define QCAMERAWINDOW_H

#include <QWidget>
#include <QCamera>
#include <QCameraInfo>
#include <QCameraViewfinder>
#include <QCameraViewfinderSettings>
#include <QMediaRecorder>
#include <QCameraImageCapture>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QScrollArea>
#include <QGroupBox>
#include <QSpinBox>
#include <QLineEdit>
#include <QTabWidget>
#include <QImage>

class QCameraWindow : public QWidget
{
    Q_OBJECT
public:
    QCameraWindow(QWidget *parent = 0);
    ~QCameraWindow();

signals:
    void signalCloseCameraWindow(QString flag);

public slots:
    void slotOpenCamera();
    void stopCameraSlot();
    void slotCameraLock();

    void slotCaptureImage();
    void slotSaveCapture();

    void slotStartRecord();
    void slotPauseRecord();
    void slotStopRecord();

    void updateCameraList();
    void updateCaptureMode(int index);

    void updateCameraState(QCamera::State state);
    void updateLockStatus(QCamera::LockStatus status, QCamera::LockChangeReason reason);
    void updateRecorderState(QMediaRecorder::State state);
    void updateRecordTime(qint64 duration);

    void readyForCapture(bool ready);
    void processCapturedImage(int requestId, const QImage& img);

    void displayCameraError();
    void displayRecorderError();
    void displayCaptureError(int, QCameraImageCapture::Error, const QString &errorString);

protected:
    void closeEvent(QCloseEvent *event);

private:

    void setCamera(const QCameraInfo &cameraInfo);
    void freeCamera();

    void setRecord();
    void freeRecord();

    void setCapture();
    void freeCapture();

private:

    QLabel *cameraInfoLabel;
    QComboBox *cameraInfoBox;
    QPushButton *updateCameraButton;

    QGroupBox *cameraParamBox;
    QLabel *showWidthLabel;
    QSpinBox *showWidthBox;
    QLabel *showHeightLabel;
    QSpinBox *showHeightBox;
    QPushButton *startCameraButton;
    QPushButton *stopCameraButton;
    QPushButton *focusCameraButton;

    QCameraViewfinder *viewfinder;
    QScrollArea *scrollArea;

    QTabWidget *tabweight;
    QWidget *imageWidget;
    QWidget *videoWidget;

    QPushButton *captureImageButton;
    QPushButton *saveImageButton;
    QLabel *captureImageLable;

    QPushButton *startRecordButton;
    QPushButton *pauseRecordButton;
    QPushButton *stopRecordButton;
    QLabel *recordTimeLable;

private:

    QCamera *camera;
    QMediaRecorder *videoRecord;
    QCameraImageCapture *imageCapture;

    QCameraViewfinderSettings viewfinderSettings;
    QList<QCameraInfo> cameraList;

    QString savePath;
    QImage captureImage;

private:

    void init();
    void initUI();
    void initConnect();
};

#endif // QCAMERAWINDOW_H
