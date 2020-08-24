#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "videocroppingwindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>

VideoCroppingWindow::VideoCroppingWindow(QWidget *parent) : QWidget(parent)
{
    init();
    initUI();
    initConnect();
}

VideoCroppingWindow::~VideoCroppingWindow()
{
    if(croppingVideo)
    {
        delete croppingVideo;
        croppingVideo = NULL;
    }
}

void VideoCroppingWindow::closeEvent(QCloseEvent *event)
{
    if(croppingVideo->isRunning())
    {
        QMessageBox::StandardButton button;
        button=QMessageBox::question(this,tr("视频剪辑程序"),QString(tr("视频剪辑程序正在运行，是否退出？")),
                                     QMessageBox::Yes|QMessageBox::No);
        if(button==QMessageBox::No)
        {
            event->ignore();
        }
        else if(button==QMessageBox::Yes)
        {
            croppingVideo->stopThread();
            closeCurrentVideo();
            event->accept();
            QWidget::closeEvent(event);
            emit signalCloseVideoCroppingWindow("videoCropping");
        }
    }
    else
    {
        event->accept();
        QWidget::closeEvent(event);
        emit signalCloseVideoCroppingWindow("videoCropping");
    }
}

void VideoCroppingWindow::slotOpen()
{
    QString name = QFileDialog::getOpenFileName(this,tr("open video dialog"),".","video files(*.avi *.mp4 *.mpg *.mov)");
    path = name;
    this->pathText->setText(path);
    if(path.trimmed().isEmpty())
    {
        qDebug()<<"打开的视频文件路径有误:"<<path<<endl;
        return;
    }
    videoProcess->closeVideo();
    if(videoProcess->openVideo(path)==0)
    {
        int countFrame=videoProcess->getFrameCount();
        int countSec=countFrame/videoProcess->getFrameFPS();

        startPosBox->setMaximum(countSec);
        stopPosBox->setMaximum(countSec);

        openButton->setEnabled(true);
        startButton->setEnabled(true);
        stopButton->setEnabled(false);
    }
}

void VideoCroppingWindow::slotIsProcessAll(bool isChecked)
{
    videoPosBox->setEnabled(!isChecked);
}

void VideoCroppingWindow::slotStart()
{
    QFileInfo info(path);
    qDebug()<<info.absolutePath();
    if(croppingVideo->initSaveCroppingVideoData(info.absolutePath(),videoProcess,scaleSizeBox->value(),isAllCropBox->isChecked(),startPosBox->value(),stopPosBox->value())==0)
    {
        croppingVideo->startThread();
        croppingVideo->start();
        stopButton->setEnabled(true);
        startButton->setEnabled(false);
        openButton->setEnabled(false);
    }
}

void VideoCroppingWindow::slotStop()
{
    croppingVideo->stopThread();
    closeCurrentVideo();
    stopButton->setEnabled(false);
    startButton->setEnabled(false);
    openButton->setEnabled(true);
    slotIsProcessAll(isAllCropBox->isChecked());
}

void VideoCroppingWindow::slotFinish(QString name)
{
    closeCurrentVideo();
    qDebug() << "save video:" << name;
    stopButton->setEnabled(false);
    startButton->setEnabled(false);
    openButton->setEnabled(true);
    slotIsProcessAll(isAllCropBox->isChecked());
}

void VideoCroppingWindow::init()
{
    videoProcess = std::shared_ptr<VideoProcess>(new VideoProcess());
    croppingVideo = new CroppingVideoThread();
    path = "";
}

void VideoCroppingWindow::initUI()
{
    QHBoxLayout *layout1=new QHBoxLayout();
    isAllCropBox=new QCheckBox(tr("是否针对全部视频"));
    isAllCropBox->setChecked(true);

    scaleSizeLabel=new QLabel(tr("视频缩放尺寸："));
    scaleSizeBox=new QDoubleSpinBox();
    scaleSizeBox->setMinimum(0.00001);
    scaleSizeBox->setMaximum(10);
    scaleSizeBox->setValue(1.0);
    scaleSizeBox->setSingleStep(0.1);

    layout1->addWidget(isAllCropBox);
    layout1->addWidget(scaleSizeLabel);
    layout1->addWidget(scaleSizeBox);

    QHBoxLayout *videoPosLayout = new QHBoxLayout();
    videoPosLayout->setSpacing(20);
    videoPosLayout->setContentsMargins(0, 0, 0, 0);

    startLabel = new QLabel(tr("视频开始裁剪位置："));
    startPosBox = new QSpinBox();
    startPosBox->setSuffix("s");
    startPosBox->setMinimum(0);
    startPosBox->setValue(0);

    stopLabel = new QLabel(tr("视频结束裁剪位置："));
    stopPosBox = new QSpinBox();
    stopPosBox->setSuffix("s");
    stopPosBox->setMinimum(0);
    stopPosBox->setValue(0);

    videoPosLayout->addWidget(startLabel);
    videoPosLayout->addWidget(startPosBox);
    videoPosLayout->addWidget(stopLabel);
    videoPosLayout->addWidget(stopPosBox);

    videoPosBox = new QGroupBox();
    videoPosBox->setLayout(videoPosLayout);
    videoPosBox->setEnabled(false);

    QHBoxLayout *layout3=new QHBoxLayout();
    pathText=new QLineEdit();
    pathText->setReadOnly(true);
    openButton=new QPushButton(tr("打开视频"));

    layout3->addWidget(pathText);
    layout3->addWidget(openButton);

    QHBoxLayout *layout4=new QHBoxLayout();
    layout4->setSpacing(50);
    startButton=new QPushButton(tr("开始剪辑"));
    stopButton=new QPushButton(tr("结束剪辑"));
    startButton->setEnabled(false);
    stopButton->setEnabled(false);

    layout4->addWidget(startButton);
    layout4->addWidget(stopButton);

    QHBoxLayout *layout5=new QHBoxLayout();
    infoLabel=new QLabel(tr("备注：0.1~1.0为缩小视频帧\n      1.0~10.0为放大视频帧"));
    infoLabel->setFixedHeight(40);
    layout5->addWidget(infoLabel);

    mainLayout = new QVBoxLayout();
    mainLayout->setSpacing(15);

    mainLayout->addLayout(layout1);
    mainLayout->addWidget(videoPosBox);
    mainLayout->addLayout(layout3);
    mainLayout->addLayout(layout4);
    mainLayout->addLayout(layout5);

    this->setLayout(mainLayout);
    //this->setMaximumSize(700,520);
    this->setMinimumSize(600, 300);
    this->setWindowTitle(tr("视频剪辑"));
}

void VideoCroppingWindow::initConnect()
{
    connect(openButton,&QPushButton::clicked,this,&VideoCroppingWindow::slotOpen);
    connect(isAllCropBox, &QCheckBox::clicked, this, &VideoCroppingWindow::slotIsProcessAll);
    connect(startButton,&QPushButton::clicked,this,&VideoCroppingWindow::slotStart);
    connect(stopButton,&QPushButton::clicked,this,&VideoCroppingWindow::slotStop);
    connect(croppingVideo,&CroppingVideoThread::signalFinish,this,&VideoCroppingWindow::slotFinish);
}

void VideoCroppingWindow::closeCurrentVideo()
{
    croppingVideo->wait();
    videoProcess->closeVideo();
}
