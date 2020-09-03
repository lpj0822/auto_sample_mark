#pragma execution_character_set("utf-8")
#include "qcamerawindow.h"
#include <QCloseEvent>
#include <QPixmap>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFileDialog>

QCameraWindow::QCameraWindow(QWidget *parent) : QWidget(parent)
{
    init();
    initUI();
    initConnect();
}

QCameraWindow::~QCameraWindow()
{
    freeRecord();
    freeCapture();
    freeCamera();
}

void QCameraWindow::closeEvent(QCloseEvent *event)
{
    if(camera != NULL && camera->isAvailable())
    {
        QMessageBox::StandardButton button;
        button = QMessageBox::question(this,tr("视频采集"),QString(tr("摄像头正在采集数据，是否退出？")),
                                     QMessageBox::Yes|QMessageBox::No);
        if(button==QMessageBox::No)
        {
            event->ignore();
        }
        else if(button==QMessageBox::Yes)
        {
            stopCameraSlot();
            event->accept();
            QWidget::closeEvent(event);
            emit signalCloseCameraWindow("camera");
        }
    }
    else
    {
        event->accept();
        QWidget::closeEvent(event);
        emit signalCloseCameraWindow("camera");
    }
}

void QCameraWindow::slotOpenCamera()
{
    freeRecord();
    freeCapture();
    freeCamera();
    setCamera(cameraList[cameraInfoBox->currentData().toInt()]);
    setRecord();
    setCapture();
}

void QCameraWindow::stopCameraSlot()
{
    if(camera != NULL)
    {
        if(camera->isAvailable())
        {
            camera->stop();
            freeRecord();
            freeCapture();
            freeCamera();
        }
    }
    startCameraButton->setEnabled(true);
    stopCameraButton->setEnabled(false);
    focusCameraButton->setEnabled(false);
    tabweight->setEnabled(false);
}

void QCameraWindow::slotCameraLock()
{
    if(camera != NULL)
    {
        switch (camera->lockStatus())
        {
        case QCamera::Searching:
        case QCamera::Locked:
            camera->unlock();
            break;
        case QCamera::Unlocked:
            camera->searchAndLock();
            break;
        }
    }
}

void QCameraWindow::slotCaptureImage()
{
    if(imageCapture != NULL)
    {
        imageCapture->capture();
    }
}

void QCameraWindow::slotSaveCapture()
{
    if(!captureImage.isNull())
    {
        QString filename = QFileDialog::getSaveFileName(this, tr("保存图片"), savePath, tr("Images (*.png *.jpg)"));
        if(!filename.isNull())
        {
            savePath = filename;
            captureImage.save(filename);
            captureImage = QImage();
            saveImageButton->setEnabled(false);
        }
    }
}

void QCameraWindow::slotStartRecord()
{
    if(videoRecord != NULL)
    {
        videoRecord->record();
        updateRecordTime(0);
    }
}

void QCameraWindow::slotPauseRecord()
{
    if(videoRecord != NULL)
    {
        videoRecord->pause();
    }
}

void QCameraWindow::slotStopRecord()
{
    if(videoRecord != NULL)
    {
        videoRecord->stop();
    }
}

void QCameraWindow::updateCameraList()
{
    int count = 0;
    cameraList.clear();
    cameraInfoBox->clear();
    cameraList = QCameraInfo::availableCameras();
    foreach (const QCameraInfo &cameraInfo, cameraList)
    {
        cameraInfoBox->addItem(cameraInfo.deviceName(), count);
        count++;
    }

    if (cameraList.size() <= 0)
    {
        cameraParamBox->setEnabled(false);
    }
    else
    {
        startCameraButton->setEnabled(true);
        stopCameraButton->setEnabled(false);
        focusCameraButton->setEnabled(false);
    }
    tabweight->setEnabled(false);
}

void QCameraWindow::updateCameraState(QCamera::State state)
{
    switch (state)
    {
    case QCamera::ActiveState:
        updateCameraButton->setEnabled(false);
        startCameraButton->setEnabled(false);
        stopCameraButton->setEnabled(true);
        focusCameraButton->setEnabled(true);
        tabweight->setEnabled(true);
        break;
    case QCamera::UnloadedState:
    case QCamera::LoadedState:
        updateCameraButton->setEnabled(true);
        startCameraButton->setEnabled(true);
        stopCameraButton->setEnabled(false);
        focusCameraButton->setEnabled(false);
        tabweight->setEnabled(false);
        break;
    }
}

void QCameraWindow::updateLockStatus(QCamera::LockStatus status, QCamera::LockChangeReason reason)
{
    QColor indicationColor = Qt::black;

    switch (status) {
    case QCamera::Searching:
        indicationColor = Qt::yellow;
        focusCameraButton->setText(tr("正在聚焦"));
        break;
    case QCamera::Locked:
        indicationColor = Qt::darkGreen;
        focusCameraButton->setText(tr("未聚焦"));
        break;
    case QCamera::Unlocked:
        indicationColor = reason == QCamera::LockFailed ? Qt::red : Qt::green;
        focusCameraButton->setText(tr("已聚焦"));
        if (reason == QCamera::LockFailed)
        {
            QMessageBox::warning(this, tr("摄像头聚焦"), tr("摄像头聚焦失败!"));
        }
    }

    focusCameraButton->setStyleSheet(QString("background-color:%1").arg(indicationColor.name()));
}

void QCameraWindow::updateRecorderState(QMediaRecorder::State state)
{
    switch (state) {
    case QMediaRecorder::StoppedState:
        startRecordButton->setEnabled(true);
        pauseRecordButton->setEnabled(false);
        stopRecordButton->setEnabled(false);
        break;
    case QMediaRecorder::PausedState:
        startRecordButton->setEnabled(true);
        pauseRecordButton->setEnabled(false);
        stopRecordButton->setEnabled(true);
        break;
    case QMediaRecorder::RecordingState:
        startRecordButton->setEnabled(false);
        pauseRecordButton->setEnabled(true);
        stopRecordButton->setEnabled(true);
        break;
    }
}

void QCameraWindow::updateRecordTime(qint64 duration)
{
    recordTimeLable->setText(tr("录制时长:%1s").arg(duration / 1000));
}

void QCameraWindow::readyForCapture(bool ready)
{
    captureImageButton->setEnabled(ready);
}

void QCameraWindow::processCapturedImage(int requestId, const QImage& img)
{
    captureImage = img.scaled(viewfinder->size(),
                                    Qt::KeepAspectRatio,
                                    Qt::SmoothTransformation);
    QPixmap showImage = QPixmap::fromImage(captureImage).scaled(captureImageLable->width(), captureImageLable->height(),
                                                                Qt::KeepAspectRatio, Qt::SmoothTransformation);
    captureImageLable->setPixmap(showImage);
    saveImageButton->setEnabled(true);
}

void QCameraWindow::displayCameraError()
{
    QMessageBox::warning(this, tr("摄像头打开失败"), camera->errorString());
    startCameraButton->setEnabled(true);
    stopCameraButton->setEnabled(false);
    focusCameraButton->setEnabled(false);
    tabweight->setEnabled(false);
}

void QCameraWindow::displayRecorderError()
{
    QMessageBox::warning(this, tr("视频录制初始化失败"), videoRecord->errorString());
    videoWidget->setEnabled(false);
}

void QCameraWindow::displayCaptureError(int id, const QCameraImageCapture::Error error, const QString &errorString)
{
    Q_UNUSED(id);
    Q_UNUSED(error);
    QMessageBox::warning(this, tr("图象采集初始化失败"), errorString);
    imageWidget->setEnabled(false);
}

void QCameraWindow::setCamera(const QCameraInfo &cameraInfo)
{
    camera = new QCamera(cameraInfo);

    connect(camera, &QCamera::stateChanged, this, &QCameraWindow::updateCameraState);
    connect(camera, SIGNAL(error(QCamera::Error)), this, SLOT(displayCameraError()));
    connect(camera, SIGNAL(lockStatusChanged(QCamera::LockStatus,QCamera::LockChangeReason)),
            this, SLOT(updateLockStatus(QCamera::LockStatus,QCamera::LockChangeReason)));

    viewfinder->setFixedSize(showWidthBox->value(), showHeightBox->value());
    camera->setCaptureMode(QCamera::CaptureViewfinder);
    camera->setViewfinder(viewfinder);

    updateCameraState(camera->state());
    updateLockStatus(camera->lockStatus(), QCamera::UserRequest);

    updateCaptureMode(0);
    camera->start();
}

void QCameraWindow::freeCamera()
{
    if(camera != NULL)
    {
        disconnect(camera, &QCamera::stateChanged, this, &QCameraWindow::updateCameraState);
        disconnect(camera, SIGNAL(error(QCamera::Error)), this, SLOT(displayCameraError()));
        disconnect(camera, SIGNAL(lockStatusChanged(QCamera::LockStatus,QCamera::LockChangeReason)),
                this, SLOT(updateLockStatus(QCamera::LockStatus,QCamera::LockChangeReason)));
        delete camera;
        camera = NULL;
    }
}

void QCameraWindow::setRecord()
{
    if(camera != NULL)
    {
        videoRecord = new QMediaRecorder(camera);
        connect(videoRecord, &QMediaRecorder::stateChanged, this, &QCameraWindow::updateRecorderState);
        connect(videoRecord, &QMediaRecorder::durationChanged, this, &QCameraWindow::updateRecordTime);
        connect(videoRecord, SIGNAL(error(QMediaRecorder::Error)), this, SLOT(displayRecorderError()));
        updateRecorderState(videoRecord->state());
    }
}

void QCameraWindow::freeRecord()
{
    if(videoRecord != NULL)
    {
        disconnect(videoRecord, &QMediaRecorder::stateChanged, this, &QCameraWindow::updateRecorderState);
        disconnect(videoRecord, &QMediaRecorder::durationChanged, this, &QCameraWindow::updateRecordTime);
        disconnect(videoRecord, SIGNAL(error(QMediaRecorder::Error)), this, SLOT(displayRecorderError()));
        delete videoRecord;
        videoRecord = NULL;
    }
}

void QCameraWindow::setCapture()
{
    if(camera != NULL)
    {
        imageCapture = new QCameraImageCapture(camera);
        connect(imageCapture, &QCameraImageCapture::readyForCaptureChanged, this, &QCameraWindow::readyForCapture);
        connect(imageCapture, &QCameraImageCapture::imageCaptured, this, &QCameraWindow::processCapturedImage);
        connect(imageCapture, SIGNAL(error(int,QCameraImageCapture::Error,QString)), this,
                SLOT(displayCaptureError(int,QCameraImageCapture::Error,QString)));
    }
}

void QCameraWindow::freeCapture()
{
    if(imageCapture != NULL)
    {
        disconnect(imageCapture, &QCameraImageCapture::readyForCaptureChanged, this, &QCameraWindow::readyForCapture);
        disconnect(imageCapture, &QCameraImageCapture::imageCaptured, this, &QCameraWindow::processCapturedImage);
        disconnect(imageCapture, SIGNAL(error(int,QCameraImageCapture::Error,QString)), this,
                SLOT(displayCaptureError(int,QCameraImageCapture::Error,QString)));
        delete imageCapture;
        imageCapture = NULL;
    }
}

void QCameraWindow::updateCaptureMode(int index)
{
    QCamera::CaptureModes captureMode = index == 0 ? QCamera::CaptureStillImage : QCamera::CaptureVideo;
    if (camera->isCaptureModeSupported(captureMode))
    {
        camera->setCaptureMode(captureMode);
        if(index == 0)
        {
            imageWidget->setEnabled(true);
            saveImageButton->setEnabled(false);
        }
        else if(index == 1)
        {
            imageWidget->setEnabled(true);
        }
    }
    else
    {
        if(index == 0)
            imageWidget->setEnabled(false);
        else if(index == 1)
            imageWidget->setEnabled(false);
    }
}

void QCameraWindow::init()
{
    camera = NULL;
    videoRecord = NULL;
    imageCapture = NULL;
    cameraList.clear();
    savePath = "./image.png";
    captureImage = QImage();

    viewfinderSettings.setMinimumFrameRate(1.0);
    viewfinderSettings.setMaximumFrameRate(60.0);
}

void QCameraWindow::initUI()
{
    cameraInfoLabel = new QLabel(tr("摄像头列表："));
    cameraInfoBox = new QComboBox();
    updateCameraButton = new QPushButton(tr("刷新"));

    QHBoxLayout *topLayout=new QHBoxLayout();
    topLayout->setSpacing(20);
    topLayout->setAlignment(Qt::AlignCenter);
    topLayout->addWidget(cameraInfoLabel);
    topLayout->addWidget(cameraInfoBox);
    topLayout->addWidget(updateCameraButton);

    cameraParamBox = new QGroupBox();
    showWidthLabel = new QLabel(tr("预览图象宽度："));
    showWidthBox = new QSpinBox();
    showWidthBox->setMinimum(10);
    showWidthBox->setMaximum(5000);
    showWidthBox->setSingleStep(100);
    showWidthBox->setValue(1280);
    showHeightLabel = new QLabel(tr("预览图象高度："));
    showHeightBox = new QSpinBox();
    showHeightBox->setMinimum(10);
    showHeightBox->setMaximum(5000);
    showHeightBox->setSingleStep(100);
    showHeightBox->setValue(720);
    startCameraButton = new QPushButton(tr("打开摄像头"));
    stopCameraButton = new QPushButton(tr("关闭摄像头"));
    focusCameraButton = new QPushButton(tr("摄像头聚焦"));

    QVBoxLayout *cameraParam = new QVBoxLayout();
    cameraParam->setAlignment(Qt::AlignCenter);
    QHBoxLayout *layout1 = new QHBoxLayout();
    layout1->setSpacing(20);
    layout1->addWidget(showWidthLabel);
    layout1->addWidget(showWidthBox);
    layout1->addWidget(showHeightLabel);
    layout1->addWidget(showHeightBox);
    QHBoxLayout *layout2 = new QHBoxLayout();
    layout2->setSpacing(30);
    layout2->addWidget(startCameraButton);
    layout2->addWidget(stopCameraButton);
    layout2->addWidget(focusCameraButton);
    cameraParam->addLayout(layout1);
    cameraParam->addLayout(layout2);
    cameraParamBox->setLayout(cameraParam);

    viewfinder = new QCameraViewfinder(this);
    scrollArea = new QScrollArea();
    scrollArea->setAlignment(Qt::AlignCenter);
    scrollArea->setBackgroundRole(QPalette::Dark);
    scrollArea->setWidget(viewfinder);

    imageWidget = new QWidget();
    captureImageButton = new QPushButton(tr("拍照"));
    saveImageButton = new QPushButton(tr("保存"));
    captureImageLable = new QLabel();
    captureImageLable->setScaledContents(true);
    imageWidget->setEnabled(false);

    QVBoxLayout *imageLayout = new QVBoxLayout();
    imageLayout->setSpacing(20);
    imageLayout->addWidget(captureImageButton);
    imageLayout->addWidget(saveImageButton);
    imageLayout->addWidget(captureImageLable);
    imageWidget->setLayout(imageLayout);

    videoWidget = new QWidget();
    startRecordButton = new QPushButton(tr("开始录制"));
    pauseRecordButton = new QPushButton(tr("暂停录制"));
    stopRecordButton = new QPushButton(tr("停止录制"));
    recordTimeLable = new QLabel(tr("录制时长:0s"));
    videoWidget->setEnabled(false);

    QVBoxLayout *videoLayout = new QVBoxLayout();
    videoLayout->setSpacing(20);
    videoLayout->addWidget(startRecordButton);
    videoLayout->addWidget(pauseRecordButton);
    videoLayout->addWidget(stopRecordButton);
    videoLayout->addWidget(recordTimeLable);
    videoWidget->setLayout(videoLayout);

    tabweight = new QTabWidget();
    tabweight->addTab(imageWidget, tr("图片采集"));
    tabweight->addTab(videoWidget, tr("视频采集"));

    QHBoxLayout *centerLayout=new QHBoxLayout();
    centerLayout->setSpacing(50);
    centerLayout->setAlignment(Qt::AlignCenter);
    centerLayout->addWidget(scrollArea);
    centerLayout->addWidget(tabweight);
    centerLayout->setStretchFactor(scrollArea, 4);
    centerLayout->setStretchFactor(tabweight, 1);

    QVBoxLayout *mainLayout = new QVBoxLayout();//主布局
    mainLayout->addLayout(topLayout);
    mainLayout->addWidget(cameraParamBox);
    mainLayout->addLayout(centerLayout);

    updateCameraList();

    this->setLayout(mainLayout);
    //this->setMaximumSize(700,520);
    this->setMinimumSize(700,520);
    this->setWindowTitle(tr("视频采集"));
}

void QCameraWindow::initConnect()
{
    connect(updateCameraButton, &QPushButton::clicked, this, &QCameraWindow::updateCameraList);
    connect(startCameraButton, &QPushButton::clicked, this, &QCameraWindow::slotOpenCamera);
    connect(stopCameraButton, &QPushButton::clicked, this, &QCameraWindow::stopCameraSlot);
    connect(focusCameraButton, &QPushButton::clicked, this, &QCameraWindow::slotCameraLock);
    connect(tabweight, &QTabWidget::currentChanged, this, &QCameraWindow::updateCaptureMode);
    connect(captureImageButton, &QPushButton::clicked, this, &QCameraWindow::slotCaptureImage);
    connect(saveImageButton, &QPushButton::clicked, this, &QCameraWindow::slotSaveCapture);
    connect(startRecordButton, &QPushButton::clicked, this, &QCameraWindow::slotStartRecord);
    connect(pauseRecordButton, &QPushButton::clicked, this, &QCameraWindow::slotPauseRecord);
    connect(stopRecordButton, &QPushButton::clicked, this, &QCameraWindow::slotStopRecord);
}
