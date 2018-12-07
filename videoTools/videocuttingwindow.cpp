#pragma execution_character_set("utf-8")
#include "videocuttingwindow.h"
#include <QDebug>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QDir>
#include <QFileInfo>
#include <QFile>
#include <QTextStream>
#include <QIcon>
#include <QMenu>

#include "helpers/dirprocess.h"
#include "videoparameterwindow.h"

VideoCuttingWindow::VideoCuttingWindow(QWidget *parent) : QWidget(parent)
{
    init();
    initUI();
    initMenu();
    initConnect();
}

VideoCuttingWindow::~VideoCuttingWindow()
{
    if(croppingVideo)
    {
        delete croppingVideo;
        croppingVideo=NULL;
    }
}

void VideoCuttingWindow::closeEvent(QCloseEvent *event)
{
    if(croppingVideo->isRunning())
    {
        QMessageBox::StandardButton button;
        button=QMessageBox::question(this,tr("视频剪切为短视频程序"),QString(tr("视频剪切为短视频程序正在运行，是否退出？")),
                                     QMessageBox::Yes|QMessageBox::No);
        if(button==QMessageBox::No)
        {
            event->ignore();
        }
        else if(button==QMessageBox::Yes)
        {
            historyProcess.writeHistoryData(this->videoPathDir, this->historyVideo);
            croppingVideo->stopThread();
            closeCurrentVideo();
            event->accept();
            QWidget::closeEvent(event);
            emit signalCloseVideoCuttingWindow("videoCutting");
        }
    }
    else
    {
        historyProcess.writeHistoryData(this->videoPathDir, this->historyVideo);
        event->accept();
        QWidget::closeEvent(event);
        emit signalCloseVideoCuttingWindow("videoCutting");
    }
}

void VideoCuttingWindow::contextMenuEvent (QContextMenuEvent * event)
{
    QMenu* popMenu = new QMenu(this);
    if(videoListWidget->itemAt(videoListWidget->mapFromGlobal(QCursor::pos())) != NULL) //如果有item则添加"修改"菜单
    {
        if (!isProcessDirBox->isChecked() && videoListWidget->currentItem()->data(Qt::UserRole).toInt() > 0)
        {
            if(currentProcessVideo.trimmed().isEmpty())
            {
                popMenu->addAction(startVideoProcessAction);
                popMenu->addAction(stopVideoProcessAction);
                startVideoProcessAction->setEnabled(true);
                stopVideoProcessAction->setEnabled(false);
            }
            else if(currentProcessVideo.startsWith(videoListWidget->currentItem()->text()))
            {
                popMenu->addAction(startVideoProcessAction);
                popMenu->addAction(stopVideoProcessAction);
                startVideoProcessAction->setEnabled(false);
                stopVideoProcessAction->setEnabled(true);
            }
        }
    }
    // 菜单出现的位置为当前鼠标的位置
    popMenu->exec(QCursor::pos());
    QWidget::contextMenuEvent(event);
}

void VideoCuttingWindow::slotOpenDir()
{
    DirProcess dirProcess;
    videoPathDir = QFileDialog::getExistingDirectory(this, tr("选择文件夹"), videoPathDir, QFileDialog::ShowDirsOnly);
    pathText->setText(this->videoPathDir);
    if(this->videoPathDir.trimmed().isEmpty() || !QDir(this->videoPathDir).exists())
    {
        qDebug() << "打开的文件路径有误:" << this->videoPathDir << endl;
    }
    else
    {
        processVideoList.clear();
        historyVideo.clear();
        QList<QString> pathList;
        pathList.clear();
        dirProcess.getDirAllFileName(this->videoPathDir, videoPostBox->currentText(), pathList);
        if (pathList.size() > 0)
        {
            historyProcess.readHistoryData(this->videoPathDir);
            initProcessVideoList(pathList);
            if(isProcessDirBox->isChecked())
            {
                startProcessButton->setEnabled(true);
                stopProcessButton->setEnabled(false);
            }
            else
            {
                startProcessButton->setEnabled(false);
                stopProcessButton->setEnabled(false);
            }
        }
        else
        {
            QMessageBox::information(this, tr("视频数据信息"), tr("%1中没有%2视频").arg(this->videoPathDir).arg(videoPostBox->currentText()));
        }
        updateListBox();
    }
}

void VideoCuttingWindow::slotStart()
{
    currentProcessVideo = videoListWidget->currentItem()->text();
    commandText->append(QString("Process video: %1").arg(currentProcessVideo));
    if(videoProcess->openVideo(currentProcessVideo) == 0)
    {
        QFileInfo info(currentProcessVideo);
        currentVideoName = info.completeBaseName();
        currentVideoSaveDir = info.absolutePath() + "/" + currentVideoName;
        cutNumber = 0;
        countMesc = videoProcess->getFrameCount() / videoProcess->getFrameFPS();
        this->videoProgressBar->setRange(0, countMesc);
        videoStartPos = 0;
        videoStopPos = std::min(videoStartPos + skipTimeBox->value(), countMesc);
        startCutVideo();
    }
    else
    {
        commandText->append(QString("Open video %1 fail!").arg(currentProcessVideo));
    }
}

void VideoCuttingWindow::slotStop()
{
    croppingVideo->stopThread();
    closeCurrentVideo();
    commandText->append(QString("Stop process video %1!").arg(currentProcessVideo));
    if(isProcessDirBox->isChecked() && processVideoList.size() > 0)
    {
        startProcessButton->setEnabled(true);
        stopProcessButton->setEnabled(false);
        centerGroundBox->setEnabled(true);
    }
    else
    {
        startProcessButton->setEnabled(false);
        stopProcessButton->setEnabled(false);
        centerGroundBox->setEnabled(true);
    }
    currentProcessVideo = "";
}

void VideoCuttingWindow::slotFinish(QString name)
{
    croppingVideo->wait();
    commandText->append(QString("Save video: %1").arg(name));
    if(videoStopPos >= countMesc)
    {
        commandText->append(QString("Process video %1 finish!").arg(currentProcessVideo));
        historyVideo.append(currentProcessVideo);
        processVideoList.removeOne(currentProcessVideo);
        updateListBox();
    }
    updateProgressBar();
    updateProcessVideo();
}

void VideoCuttingWindow::slotIsProcessDir(bool isChecked)
{
    if(processVideoList.size() > 0)
    {
        if(isChecked)
        {
            startProcessButton->setEnabled(true);
            stopProcessButton->setEnabled(false);
        }
        else
        {
            startProcessButton->setEnabled(false);
            stopProcessButton->setEnabled(false);
        }
    }
}

void VideoCuttingWindow::initProcessVideoList(const QList<QString>& pathList)
{
    for(int index = 0; index < pathList.size(); index++)
    {
        if(historyVideo.size() > 0)
        {
            if(historyVideo.indexOf(pathList[index]) == -1)
            {
                processVideoList.append(pathList[index]);
            }
        }
        else
        {
            processVideoList.append(pathList[index]);
        }
    }
}

void VideoCuttingWindow::updateProcessVideo()
{
    videoStartPos = videoStopPos;
    videoStopPos = std::min(videoStartPos + skipTimeBox->value(), countMesc);
    cutNumber++;
    if(isProcessDirBox->isChecked() && processVideoList.size() > 0)
    {
        if(videoStartPos < countMesc)
        {
            nextCutVideo();
        }
        else
        {
            slotStart();
        }
    }
    else
    {
        if(videoStartPos < countMesc)
        {
            nextCutVideo();
        }
        else
        {
            centerGroundBox->setEnabled(true);
            startProcessButton->setEnabled(false);
            stopProcessButton->setEnabled(false);
            closeCurrentVideo();
            currentProcessVideo = "";
            historyProcess.writeHistoryData(this->videoPathDir, this->historyVideo);
        }
    }
}

void VideoCuttingWindow::updateListBox()
{
    int index1 = 0;
    int index2 = 0;
    videoListWidget->clear();
    for(index1 = 0; index1 < historyVideo.size(); index1++)
    {
        QListWidgetItem *item = new QListWidgetItem(QIcon(":/qss_icons/style/rc/checkbox_checked_disabled.png"),
                                                    historyVideo[index1]);
        item->setData(Qt::UserRole, 0);
        videoListWidget->insertItem(index1, item);
    }
    for(index2 = 0; index2 < processVideoList.size(); index2++)
    {
        QListWidgetItem *item = new QListWidgetItem(QIcon(":/qss_icons/style/rc/checkbox_checked_focus.png"),
                                                    processVideoList[index2]);
        item->setData(Qt::UserRole, 1);
        videoListWidget->insertItem(index2 + index1, item);
        //videoListWidget->closePersistentEditor(item);
    }
    if(processVideoList.size() >0)
    {
        videoListWidget->setCurrentRow(index1);
    }
}

void VideoCuttingWindow::updateProgressBar()
{
    videoProgressBar->setValue(videoStopPos);
}

void VideoCuttingWindow::startCutVideo()
{
    QString saveVideoName = currentVideoName + QString("_%1%2").arg(cutNumber).arg(postBox->currentText());
    if(croppingVideo->initSaveCroppingVideoData(currentVideoSaveDir, saveVideoName,
                                                videoProcess, videoStartPos, videoStopPos)==0)
    {
        croppingVideo->startThread();
        croppingVideo->start();
        if(isProcessDirBox->isChecked())
        {
            startProcessButton->setEnabled(false);
            stopProcessButton->setEnabled(true);
            centerGroundBox->setEnabled(false);
        }
        else
        {
            startProcessButton->setEnabled(false);
            stopProcessButton->setEnabled(false);
            centerGroundBox->setEnabled(false);
        }
    }
}

void VideoCuttingWindow::nextCutVideo()
{
    QString saveVideoName = currentVideoName + QString("_%1%2").arg(cutNumber).arg(postBox->currentText());
    if(croppingVideo->setNextCroppingVideoData(currentVideoSaveDir, saveVideoName, videoStopPos)==0)
    {
        croppingVideo->startThread();
        croppingVideo->start();
        if(isProcessDirBox->isChecked())
        {
            startProcessButton->setEnabled(false);
            stopProcessButton->setEnabled(true);
            centerGroundBox->setEnabled(false);
        }
        else
        {
            startProcessButton->setEnabled(false);
            stopProcessButton->setEnabled(false);
            centerGroundBox->setEnabled(false);
        }
    }
}

void VideoCuttingWindow::closeCurrentVideo()
{
    croppingVideo->wait();
    videoProcess->closeVideo();
}

void VideoCuttingWindow::init()
{
    videoProcess = std::shared_ptr<VideoProcess>(new VideoProcess());
    croppingVideo = new CroppingVideoThread();
    processVideoList.clear();
    historyVideo.clear();
    videoPathDir = ".";
    currentProcessVideo = "";
    currentVideoSaveDir = ".";
    currentVideoName = "";
    cutNumber = 0;
    videoStartPos = 0;
    videoStopPos = 0;
    countMesc = 0;
    isStopAll = false;
}

void VideoCuttingWindow::initUI()
{
    isProcessDirBox = new QCheckBox(tr("是否批量处理"));
    isProcessDirBox->setChecked(true);

    videoPostLabel = new QLabel(tr("处理视频格式："));
    videoPostBox = new QComboBox();
    videoPostBox->addItem(tr("*.avi"));
    videoPostBox->addItem(tr("*.mov"));
    videoPostBox->addItem(tr("*.mp4"));
    videoPostBox->addItem(tr("*.mpg"));
    videoPostBox->addItem(tr("*.*"));

    QHBoxLayout *processLayout = new QHBoxLayout();
    processLayout->addWidget(isProcessDirBox);
    processLayout->addWidget(videoPostLabel);
    processLayout->addWidget(videoPostBox);

    QHBoxLayout *layout1 = new QHBoxLayout();
    postLabel = new QLabel(tr("保存视频的格式："));
    postBox = new QComboBox();
    postBox->addItem(".avi");
    layout1->addWidget(postLabel);
    layout1->addWidget(postBox);

    QHBoxLayout *layout2 = new QHBoxLayout();
    layout2->setSpacing(30);
    skipTimeLabel = new QLabel(tr("间隔时间："));
    skipTimeBox = new QSpinBox();
    skipTimeBox->setSuffix("s");
    skipTimeBox->setMinimum(1);
    skipTimeBox->setMaximum(100000);
    skipTimeBox->setValue(60);
    skipTimeBox->setSingleStep(10);
    layout2->addWidget(skipTimeLabel);
    layout2->addWidget(skipTimeBox);

    QHBoxLayout *layout3 = new QHBoxLayout();
    layout3->addLayout(layout1);
    layout3->addLayout(layout2);

    QHBoxLayout *layout4 = new QHBoxLayout();
    pathText = new QLineEdit();
    pathText->setReadOnly(true);
    openButton = new QPushButton(tr("打开视频目录"));
    layout4->addWidget(pathText);
    layout4->addWidget(openButton);

    QVBoxLayout* leftTopLayout = new QVBoxLayout();
    leftTopLayout->addLayout(processLayout);
    leftTopLayout->addLayout(layout3);
    leftTopLayout->addLayout(layout4);

    centerGroundBox = new QGroupBox(tr("参数配置"));
    centerGroundBox->setLayout(leftTopLayout);

    QHBoxLayout *layout5=new QHBoxLayout();
    layout5->setSpacing(200);
    startProcessButton = new QPushButton(tr("开始转换"));
    stopProcessButton = new QPushButton(tr("结束转换"));
    startProcessButton->setEnabled(false);
    stopProcessButton->setEnabled(false);
    layout5->addWidget(startProcessButton);
    layout5->addWidget(stopProcessButton);

    videoProgressLabel = new QLabel(tr("视频处理进度："));
    videoProgressBar = new QProgressBar();
    QHBoxLayout *layout6 = new QHBoxLayout();
    layout6->setSpacing(20);
    layout6->addWidget(videoProgressLabel);
    layout6->addWidget(videoProgressBar);
    updateProgressBar();

    QVBoxLayout *centerLayout = new QVBoxLayout();
    centerLayout->setSpacing(20);
    centerLayout->addWidget(centerGroundBox);
    centerLayout->addLayout(layout5);
    centerLayout->addLayout(layout6);

    videoListWidget = new QListWidget();

    QHBoxLayout *topLayout = new QHBoxLayout();
    topLayout->addLayout(centerLayout);
    topLayout->addWidget(videoListWidget);
    topLayout->setStretchFactor(centerLayout, 3);
    topLayout->setStretchFactor(videoListWidget, 1);

    commandText = new MyTextBrowser();
    //commandText->setFixedHeight(100);
    commandText->setReadOnly(true);

    QVBoxLayout *mainLayout = new QVBoxLayout();
    mainLayout->setMargin(10); //设置这个对话框的边距
    mainLayout->setSpacing(10);  //设置各个控件之间的边距
    mainLayout->setAlignment(Qt::AlignCenter);
    mainLayout->addLayout(topLayout);
    mainLayout->addWidget(commandText);
    mainLayout->setStretchFactor(topLayout, 5);
    mainLayout->setStretchFactor(commandText, 1);
    this->setLayout(mainLayout);
    //this->setMaximumSize(700,520);
    this->setMinimumSize(700, 600);
    this->setWindowTitle(tr("视频剪切为短视频"));
}

void VideoCuttingWindow::initMenu()
{
    startVideoProcessAction = new QAction(tr("开始转换"), this);
    stopVideoProcessAction = new QAction(tr("结束转换"), this);
}

void VideoCuttingWindow::initConnect()
{
    connect(openButton, &QPushButton::clicked, this, &VideoCuttingWindow::slotOpenDir);
    connect(startProcessButton, &QPushButton::clicked, this, &VideoCuttingWindow::slotStart);
    connect(stopProcessButton, &QPushButton::clicked, this, &VideoCuttingWindow::slotStop);

    connect(croppingVideo, &CroppingVideoThread::signalFinish, this, &VideoCuttingWindow::slotFinish);

    connect(startVideoProcessAction, &QAction::triggered, this, &VideoCuttingWindow::slotStart);
    connect(stopVideoProcessAction, &QAction::triggered, this, &VideoCuttingWindow::slotStop);

    connect(isProcessDirBox, &QCheckBox::clicked, this, &VideoCuttingWindow::slotIsProcessDir);
}
