#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "frompicturetovideowindow.h"
#include <QHBoxLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>

FromPictureToVideoWindow::FromPictureToVideoWindow(QWidget *parent) : QWidget(parent)
{
    init();
    initUI();
    initConnect();
}

FromPictureToVideoWindow::~FromPictureToVideoWindow()
{
    if(saveVideo)
    {
        delete saveVideo;
        saveVideo=NULL;
    }
}

void FromPictureToVideoWindow::slotOpen()
{
    QString post=postBox->currentText();
    pathDir = QFileDialog::getExistingDirectory(this, tr("选择文件夹"),".",QFileDialog::ShowDirsOnly);
    if(pathDir.trimmed().isEmpty())
    {
        qDebug()<<"打开的文件路径有误:"<<pathDir<<endl;
        return;
    }
    this->pathText->setText(pathDir);
    pathList.clear();
    pathList=dirProcess.getDirFileName(pathDir,post);
    startButton->setEnabled(true);
}

void FromPictureToVideoWindow::slotStart()
{
    QString fileName=pathDir+"/video.avi";
    int fps=fpsBox->value();
    if(saveVideo->initSaveVideoData(pathList,pathDir,fileName,fps)==0)
    {
        saveVideo->startThread();
        saveVideo->start();
        stopButton->setEnabled(true);
        startButton->setEnabled(false);
        openButton->setEnabled(false);
    }
}

void FromPictureToVideoWindow::slotStop()
{
    saveVideo->stopThread();
    stopButton->setEnabled(false);
    startButton->setEnabled(true);
    openButton->setEnabled(true);
}

void FromPictureToVideoWindow::slotFinish(QString name)
{
    qDebug()<<"save video:"<<name;
    stopButton->setEnabled(false);
    startButton->setEnabled(true);
    openButton->setEnabled(true);
}

void FromPictureToVideoWindow::closeEvent(QCloseEvent *event)
{
    if(saveVideo->isRunning())
    {
        QMessageBox::StandardButton button;
        button=QMessageBox::question(this,tr("图片转换为视频程序"),QString(tr("图片转换为视频程序正在运行，是否退出？")),
                                     QMessageBox::Yes|QMessageBox::No);
        if(button==QMessageBox::No)
        {
            event->ignore();
        }
        else if(button==QMessageBox::Yes)
        {
            saveVideo->stopThread();
            saveVideo->closeVideo();
            saveVideo->wait();
            event->accept();
            QWidget::closeEvent(event);
            emit signalCloseVideoFromPictureWindow("videoFromPicture");
        }
    }
    else
    {
        event->accept();
        QWidget::closeEvent(event);
        emit signalCloseVideoFromPictureWindow("videoFromPicture");
    }
}

void FromPictureToVideoWindow::init()
{
    saveVideo=new SaveVideoThread();
    pathList.clear();
    pathDir = "";
}

void FromPictureToVideoWindow::initUI()
{
    mainLayout=new QVBoxLayout();

    QHBoxLayout* layout=new QHBoxLayout();
    fpsLabel=new QLabel(tr("视频帧率："));
    fpsBox=new QSpinBox();
    fpsBox->setMinimum(1);
    fpsBox->setMaximum(1000);
    fpsBox->setSingleStep(1);
    fpsBox->setValue(25);

    postLabel=new QLabel(tr("图片格式:"));
    postBox=new QComboBox();
    postBox->addItem(tr("*.png"));
    postBox->addItem(tr("*.jpg"));
    postBox->addItem(tr("*.gif"));
    postBox->addItem(tr("*.*"));

    layout->setSpacing(50);
    layout->addWidget(fpsLabel);
    layout->addWidget(fpsBox);
    layout->addWidget(postLabel);
    layout->addWidget(postBox);

    QHBoxLayout* layout1=new QHBoxLayout();
    openButton=new QPushButton(tr("打开图片文件夹"));
    pathText=new QLineEdit();
    pathText->setReadOnly(true);
    layout1->addWidget(pathText);
    layout1->addWidget(openButton);

    QHBoxLayout*layout2=new QHBoxLayout();
    startButton=new QPushButton(tr("开始转换"));
    stopButton=new QPushButton(tr("结束转换"));
    startButton->setEnabled(false);
    stopButton->setEnabled(false);
    layout2->setSpacing(50);
    layout2->addWidget(startButton);
    layout2->addWidget(stopButton);

    mainLayout->addLayout(layout);
    mainLayout->addLayout(layout1);
    mainLayout->addLayout(layout2);
    this->setLayout(mainLayout);

    this->setMaximumSize(600, 300);
    this->setMinimumSize(600, 300);
    this->setWindowTitle(tr("图片转换为视频"));
}

void FromPictureToVideoWindow::initConnect()
{
    connect(openButton,&QPushButton::clicked,this,&FromPictureToVideoWindow::slotOpen);
    connect(startButton,&QPushButton::clicked,this,&FromPictureToVideoWindow::slotStart);
    connect(stopButton,&QPushButton::clicked,this,&FromPictureToVideoWindow::slotStop);
    connect(saveVideo,&SaveVideoThread::signalFinish,this,&FromPictureToVideoWindow::slotFinish);
}
