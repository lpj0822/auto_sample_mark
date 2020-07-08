#pragma execution_character_set("utf-8")
#include "fromvideotopicturewindow.h"
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

#include "paramWindow/videoparameterwindow.h"

FromVideoToPictureWindow::FromVideoToPictureWindow(QWidget *parent) : QWidget(parent)
{
    init();
    initUI();
    initMenu();
    initConnect();
}

FromVideoToPictureWindow::~FromVideoToPictureWindow()
{
    if(saveImage)
    {
        delete saveImage;
        saveImage = NULL;
    }
}

void FromVideoToPictureWindow::closeEvent(QCloseEvent *event)
{
    if(saveImage->isRunning())
    {
        QMessageBox::StandardButton button;
        button=QMessageBox::question(this,tr("视频转换为图片程序"),QString(tr("视频转换为图片程序正在运行，是否退出？")),
                                     QMessageBox::Yes|QMessageBox::No);
        if(button==QMessageBox::No)
        {
            event->ignore();
        }
        else if(button==QMessageBox::Yes)
        {
            historyProcess.writeHistoryData(this->videoPathDir, this->historyVideo);
            saveImage->stopThread();
            saveImage->wait();
            event->accept();
            QWidget::closeEvent(event);
            emit signalCloseVideoToPictureWindow("videoToPicture");
        }
    }
    else
    {
        historyProcess.writeHistoryData(this->videoPathDir, this->historyVideo);
        event->accept();
        QWidget::closeEvent(event);
        emit signalCloseVideoToPictureWindow("videoToPicture");
    }
}

void FromVideoToPictureWindow::contextMenuEvent (QContextMenuEvent * event)
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

void FromVideoToPictureWindow::slotOpenDir()
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
            QMessageBox::information(this, tr("视频数据信息"), tr("%1没有%2视频").arg(this->videoPathDir).arg(videoPostBox->currentText()));
        }
        updateListBox();
    }
}

void FromVideoToPictureWindow::slotStart()
{
    currentProcessVideo = videoListWidget->currentItem()->text();
    commandText->append(QString("Process video: %1").arg(currentProcessVideo));
    if(videoProcess->openVideo(currentProcessVideo) == 0)
    {
        this->videoProgressBar->setRange(0, videoProcess->getFrameCount());
        if(isProcessDirBox->isChecked())
        {
            if(saveImage->initSaveImageData(currentProcessVideo, postBox->currentText(),
                                            videoProcess, skipFrameBox->value())==0)
            {
                saveImage->startThread();
                saveImage->start();
                startProcessButton->setEnabled(false);
                stopProcessButton->setEnabled(true);
                centerGroundBox->setEnabled(false);
            }

        }
        else
        {
            int countSec = videoProcess->getFrameCount() / videoProcess->getFrameFPS();
            VideoParameterWindow *window = new VideoParameterWindow(0, countSec);
            window->setModal(true);
            int res = window->exec();
            if (res == QDialog::Accepted)
            {
                if(saveImage->initSaveImageData(currentProcessVideo, postBox->currentText(),
                                                videoProcess, skipFrameBox->value(), window->isAllSaveBox->isChecked(),
                                                window->startPosBox->value(),window->stopPosBox->value())==0)
                {
                    saveImage->startThread();
                    saveImage->start();
                    stopProcessButton->setEnabled(false);
                    startProcessButton->setEnabled(false);
                    centerGroundBox->setEnabled(false);
                }
            }
            else
            {
                commandText->append(QString("Video %1 参数未配置!").arg(currentProcessVideo));
                currentProcessVideo = "";
            }
            window->deleteLater();
        }
    }
    else
    {
        commandText->append(QString("Open video %1 fail!").arg(currentProcessVideo));
    }
}

void FromVideoToPictureWindow::slotStop()
{
    saveImage->stopThread();
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

void FromVideoToPictureWindow::slotFinish(QString name)
{
    commandText->append(QString("Save images dir: %1").arg(name));
    commandText->append(QString("Process video %1 finish!").arg(currentProcessVideo));
    historyVideo.append(currentProcessVideo);
    processVideoList.removeOne(currentProcessVideo);
    updateListBox();
    if(isProcessDirBox->isChecked() && processVideoList.size() > 0)
    {
        slotStart();
    }
    else
    {
        centerGroundBox->setEnabled(true);
        startProcessButton->setEnabled(false);
        stopProcessButton->setEnabled(false);
        currentProcessVideo = "";
        historyProcess.writeHistoryData(this->videoPathDir, this->historyVideo);
    }
}

void FromVideoToPictureWindow::slotIsProcessDir(bool isChecked)
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

void FromVideoToPictureWindow::slotVideoCurrentFrame(int number)
{
    this->currentFrameNumber = number;
    updateProgressBar();
}

void FromVideoToPictureWindow::initProcessVideoList(const QList<QString>& pathList)
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

void FromVideoToPictureWindow::updateListBox()
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

void FromVideoToPictureWindow::updateProgressBar()
{
    videoProgressBar->setValue(currentFrameNumber);
}

void FromVideoToPictureWindow::init()
{
    videoProcess = std::shared_ptr<VideoProcess>(new VideoProcess());
    saveImage = new SaveImageThread();
    processVideoList.clear();
    historyVideo.clear();
    videoPathDir = ".";
    currentProcessVideo = "";
    currentFrameNumber = 0;
    isStopAll = false;
}

void FromVideoToPictureWindow::initUI()
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

    QHBoxLayout *processLayout = new QHBoxLayout();
    processLayout->addWidget(isProcessDirBox);
    processLayout->addWidget(videoPostLabel);
    processLayout->addWidget(videoPostBox);

    QHBoxLayout *layout1 = new QHBoxLayout();
    postLabel = new QLabel(tr("保存图片的格式："));
    postBox = new QComboBox();
    postBox->addItem(".png");
    postBox->addItem(".jpg");
    layout1->addWidget(postLabel);
    layout1->addWidget(postBox);

    QHBoxLayout *layout2 = new QHBoxLayout();
    layout2->setSpacing(30);
    skipFrameLabel = new QLabel(tr("间隔帧数："));
    skipFrameBox = new QSpinBox();
    skipFrameBox->setMinimum(1);
    skipFrameBox->setValue(10);
    skipFrameBox->setSingleStep(10);
    layout2->addWidget(skipFrameLabel);
    layout2->addWidget(skipFrameBox);

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
    this->setWindowTitle(tr("视频转换为图片"));
}

void FromVideoToPictureWindow::initMenu()
{
    startVideoProcessAction = new QAction(tr("开始转换"), this);
    stopVideoProcessAction = new QAction(tr("结束转换"), this);
}

void FromVideoToPictureWindow::initConnect()
{
    connect(openButton, &QPushButton::clicked, this, &FromVideoToPictureWindow::slotOpenDir);
    connect(startProcessButton, &QPushButton::clicked, this, &FromVideoToPictureWindow::slotStart);
    connect(stopProcessButton, &QPushButton::clicked, this, &FromVideoToPictureWindow::slotStop);

    connect(saveImage,&SaveImageThread::signalCurrentFrame, this, &FromVideoToPictureWindow::slotVideoCurrentFrame);
    connect(saveImage,&SaveImageThread::signalFinish, this, &FromVideoToPictureWindow::slotFinish);

    connect(startVideoProcessAction, &QAction::triggered, this, &FromVideoToPictureWindow::slotStart);
    connect(stopVideoProcessAction, &QAction::triggered, this, &FromVideoToPictureWindow::slotStop);

    connect(isProcessDirBox, &QCheckBox::clicked, this, &FromVideoToPictureWindow::slotIsProcessDir);
}
