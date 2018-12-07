#pragma execution_character_set("utf-8")
#include "autosamplemarkwindow.h"
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
#include <QDebug>

#include "helpers/dirprocess.h"
#include "autoparamterconfig.h"

AutoSampleMarkWindow::AutoSampleMarkWindow(QWidget *parent)
    : QWidget(parent)
{
    initMainUI();
    initMenu();
    initConnect();
}

AutoSampleMarkWindow::~AutoSampleMarkWindow()
{

}

void AutoSampleMarkWindow::initData()
{
    int errorCode = 0;
    processVideoList.clear();
    historyVideo.clear();
    videoPathDir = ".";
    currentProcessVideo = "";
    currentFrameNumber = 0;
    updateProgressBar();
    errorCode = sampleMarkThread.initModel(AutoParamterConfig::getCaffeNet(), AutoParamterConfig::getCaffeModel());
    if(errorCode < 0)
    {
        startProcessButton->setEnabled(false);
        stopProcessButton->setEnabled(false);
        videoListWidget->setEnabled(false);
        centerGroundBox->setEnabled(false);
        QMessageBox::information(this, tr("检测模型初始化"), tr("检测模型初始化失败"));
    }
}

void AutoSampleMarkWindow::closeEvent(QCloseEvent *event)
{
    if(sampleMarkThread.isRunning())
    {
        QMessageBox::StandardButton button;
        button=QMessageBox::question(this,tr("自动化样本标注程序"),QString(tr("自动化样本标注程序正在运行，是否退出？")),
                                     QMessageBox::Yes|QMessageBox::No);
        if(button==QMessageBox::No)
        {
            event->ignore();
        }
        else if(button==QMessageBox::Yes)
        {
            historyProcess.writeHistoryData(this->videoPathDir, this->historyVideo);
            sampleMarkThread.stopThread();
            sampleMarkThread.wait();
            event->accept();
            QWidget::closeEvent(event);
            emit signalCloseAutoSampleMarkWindow("mark");
        }
    }
    else
    {
        historyProcess.writeHistoryData(this->videoPathDir, this->historyVideo);
        event->accept();
        QWidget::closeEvent(event);
        emit signalCloseAutoSampleMarkWindow("mark");
    }
}

void AutoSampleMarkWindow::contextMenuEvent (QContextMenuEvent * event)
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

void AutoSampleMarkWindow::slotOpenDir()
{
    DirProcess dirProcess;
    this->videoPathDir = QFileDialog::getExistingDirectory(this, tr("选择文件夹"), videoPathDir, QFileDialog::ShowDirsOnly);
    this->videoDirText->setText(this->videoPathDir);
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
            QMessageBox::information(this, tr("视频数据信息"), tr("%1没有%2视频").arg(this->videoPathDir).arg(videoPostBox->currentText()));
        }
        updateListBox();
    }
}

void AutoSampleMarkWindow::slotStart()
{
    if(isProcessDirBox->isChecked())
    {
        sampleMarkThread.initData(processVideoList, skipFrameBox->value(), confidenceThresholdBox->value());
        sampleMarkThread.startThread();
        sampleMarkThread.start();
        startProcessButton->setEnabled(false);
        stopProcessButton->setEnabled(true);
        centerGroundBox->setEnabled(false);
    }
    else
    {
        QList<QString> tempVideo;
        currentProcessVideo = videoListWidget->currentItem()->text();
        tempVideo.append(currentProcessVideo);
        sampleMarkThread.initData(tempVideo, skipFrameBox->value(), confidenceThresholdBox->value());
        sampleMarkThread.startThread();
        sampleMarkThread.start();
        centerGroundBox->setEnabled(false);
    }
}

void AutoSampleMarkWindow::slotStop()
{
    sampleMarkThread.stopThread();
    startProcessButton->setEnabled(false);
    stopProcessButton->setEnabled(false);
    centerGroundBox->setEnabled(false);
}

void AutoSampleMarkWindow::slotIsProcessDir(bool isChecked)
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

void AutoSampleMarkWindow::slotVideoInit(int allFrameCount)
{
    this->videoProgressBar->setRange(0, allFrameCount);
}

void AutoSampleMarkWindow::slotVideoCurrentFrame(int number)
{
    this->currentFrameNumber = number;
    updateProgressBar();
}

void AutoSampleMarkWindow::slotVideoFinish(QString videoInfo)
{
    commandText->append(videoInfo);
    if(videoInfo.startsWith("Video"))
    {
        QString videoName = videoInfo.mid(6);
        historyVideo.append(videoName);
        processVideoList.removeOne(videoName);
        qDebug() << "video:" << videoName << endl;
        updateListBox();
    }
    currentFrameNumber = 0;
    updateProgressBar();
}

void AutoSampleMarkWindow::slotFinishAll()
{
    centerGroundBox->setEnabled(true);
    if(isProcessDirBox->isChecked()&& processVideoList.size() > 0)
    {
        startProcessButton->setEnabled(true);
        stopProcessButton->setEnabled(false);
    }
    else
    {
        startProcessButton->setEnabled(false);
        stopProcessButton->setEnabled(false);
    }
    currentProcessVideo = "";
    historyProcess.writeHistoryData(this->videoPathDir, this->historyVideo);
}

void AutoSampleMarkWindow::initProcessVideoList(const QList<QString>& pathList)
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

void AutoSampleMarkWindow::updateListBox()
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

void AutoSampleMarkWindow::updateProgressBar()
{
    videoProgressBar->setValue(currentFrameNumber);
}

void AutoSampleMarkWindow::initMainUI()
{
    isProcessDirBox = new QCheckBox(tr("是否批量处理"));
    isProcessDirBox->setChecked(true);

    videoPostLabel = new QLabel(tr("处理视频格式："));
    videoPostBox = new QComboBox();
    videoPostBox->addItem(tr("*.mov"));
    videoPostBox->addItem(tr("*.avi"));
    videoPostBox->addItem(tr("*.mp4"));
    videoPostBox->addItem(tr("*.mpg"));
    videoPostBox->addItem(tr("*.*"));

    saveImagePostLabel = new QLabel(tr("保存图片格式："));
    saveImagePostBox = new QComboBox();
    saveImagePostBox->addItem(".png");
    saveImagePostBox->addItem(".jpg");
    saveImagePostBox->addItem(".gif");

    skipFrameLabel = new QLabel(tr("视频间隔处理帧数："));
    skipFrameBox = new QSpinBox();
    skipFrameBox->setMinimum(1);
    skipFrameBox->setValue(50);
    skipFrameBox->setMaximum(1000);

    confidenceThresholdLabel = new QLabel(tr("目标检测置信度阈值："));
    confidenceThresholdBox = new QDoubleSpinBox();
    confidenceThresholdBox->setMinimum(0.5);
    confidenceThresholdBox->setMaximum(1.0);
    confidenceThresholdBox->setValue(0.56);
    confidenceThresholdBox->setSingleStep(0.01);

    QGridLayout *centerTopLayout0 = new QGridLayout();
    centerTopLayout0->setSpacing(80);
    centerTopLayout0->addWidget(isProcessDirBox, 0, 0, 1, 1);
    centerTopLayout0->addWidget(videoPostLabel, 1, 0, 1, 1);
    centerTopLayout0->addWidget(videoPostBox, 1, 1, 1, 1);
    centerTopLayout0->addWidget(saveImagePostLabel, 1, 2, 1, 1);
    centerTopLayout0->addWidget(saveImagePostBox, 1, 3, 1, 1);
    centerTopLayout0->addWidget(skipFrameLabel, 2, 0, 1, 1);
    centerTopLayout0->addWidget(skipFrameBox, 2, 1, 1, 1);
    centerTopLayout0->addWidget(confidenceThresholdLabel, 2, 2, 1, 1);
    centerTopLayout0->addWidget(confidenceThresholdBox, 2, 3, 1, 1);

    videoDirLabel = new QLabel(tr("视频目录："));
    videoDirText = new QLineEdit();
    videoDirText->setReadOnly(true);
    openVideoDirButton = new QPushButton(tr("打开视频目录"));

    QHBoxLayout *hLayout = new QHBoxLayout();
    hLayout->setSpacing(20);
    hLayout->addWidget(videoDirLabel);
    hLayout->addWidget(videoDirText);
    hLayout->addWidget(openVideoDirButton);

    QVBoxLayout *centerTopLayout1 = new QVBoxLayout();
    centerTopLayout1->addSpacing(20);
    centerTopLayout1->addLayout(centerTopLayout0);
    centerTopLayout1->addLayout(hLayout);

    centerGroundBox = new QGroupBox(tr("参数配置"));
    centerGroundBox->setLayout(centerTopLayout1);

    startProcessButton = new QPushButton(tr("开始处理"));
    stopProcessButton = new QPushButton(tr("结束处理"));
    startProcessButton->setEnabled(false);
    stopProcessButton->setEnabled(false);

    QHBoxLayout *hLayout1 = new QHBoxLayout();
    hLayout1->setSpacing(200);
    hLayout1->addWidget(startProcessButton);
    hLayout1->addWidget(stopProcessButton);

    videoProgressLabel = new QLabel(tr("视频处理进度："));
    videoProgressBar = new QProgressBar();

    QHBoxLayout *hLayout2 = new QHBoxLayout();
    hLayout2->setSpacing(20);
    hLayout2->addWidget(videoProgressLabel);
    hLayout2->addWidget(videoProgressBar);

    QVBoxLayout *centerLayout = new QVBoxLayout();
    centerLayout->setSpacing(20);
    centerLayout->addWidget(centerGroundBox);
    centerLayout->addLayout(hLayout1);
    centerLayout->addLayout(hLayout2);

    videoListWidget = new QListWidget();

    QHBoxLayout *topLayout = new QHBoxLayout();
    topLayout->addLayout(centerLayout);
    topLayout->addWidget(videoListWidget);
    topLayout->setStretchFactor(centerLayout, 3);
    topLayout->setStretchFactor(videoListWidget, 1);

    commandText = new MyTextBrowser();
    commandText->setFixedHeight(50);
    commandText->setReadOnly(true);

    QVBoxLayout *mainLayout = new QVBoxLayout();
    mainLayout->setMargin(10); //设置这个对话框的边距
    mainLayout->setSpacing(10);  //设置各个控件之间的边距
    mainLayout->setAlignment(Qt::AlignCenter);
    mainLayout->addLayout(topLayout);
    mainLayout->addWidget(commandText);
    this->setLayout(mainLayout);
    //this->setMaximumSize(700,520);
    this->setMinimumSize(800, 600);
    this->setWindowTitle(tr("自动化样本标注"));
}

void AutoSampleMarkWindow::initMenu()
{
    startVideoProcessAction = new QAction(tr("开始处理"), this);
    stopVideoProcessAction = new QAction(tr("停止处理"), this);
}

void AutoSampleMarkWindow::initConnect()
{
    connect(openVideoDirButton, &QPushButton::clicked, this,&AutoSampleMarkWindow::slotOpenDir);
    connect(startProcessButton, &QPushButton::clicked, this,&AutoSampleMarkWindow::slotStart);
    connect(stopProcessButton, &QPushButton::clicked, this,&AutoSampleMarkWindow::slotStop);
    connect(isProcessDirBox, &QCheckBox::clicked, this, &AutoSampleMarkWindow::slotIsProcessDir);
    connect(&sampleMarkThread, &AutoSampleMarkThread::signalVideoFinish, this, &AutoSampleMarkWindow::slotVideoFinish);
    connect(&sampleMarkThread, &AutoSampleMarkThread::signalFinishAll, this, &AutoSampleMarkWindow::slotFinishAll);
    connect(&sampleMarkThread, &AutoSampleMarkThread::signalCurrentFrame, this, &AutoSampleMarkWindow::slotVideoCurrentFrame);
    connect(&sampleMarkThread, &AutoSampleMarkThread::signalVideoInit, this, &AutoSampleMarkWindow::slotVideoInit);
    connect(startVideoProcessAction, &QAction::triggered, this, &AutoSampleMarkWindow::slotStart);
    connect(stopVideoProcessAction, &QAction::triggered, this, &AutoSampleMarkWindow::slotStop);
}
