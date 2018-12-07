#pragma execution_character_set("utf-8")
#include "controlwindow.h"
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

#include "manualparamterconfig.h"
#include "videomarkparamterconfig.h"
#include "videoparameterwindow.h"

ControlWindow::ControlWindow(QWidget *parent)
    : QWidget(parent)
{
    initData();
    initUI();
    initConnect();
}

ControlWindow::~ControlWindow()
{
    if(videoMultipletracking != NULL)
    {
        delete videoMultipletracking;
        videoMultipletracking = NULL;
    }
}

void ControlWindow::setMarkDataList(const QString markDataDir, const QList<QString> markDataList, const MarkDataType dataType)
{
    if(this->isMark)
    {
        saveMarkDataResult();
    }
    writeMarkHistory();
    initMarkData(markDataDir, dataType);
    initImageData();
    initVideoData();

    updateIsMarkButton(this->isMark);
    updateDrawLabel(this->isMark);

    if(markDataList.size() > 0)
    {
        this->processMarkDataList = markDataList;
        initImageList();
        updateListBox();
        updateMarkProcessLable();
        setMarkDataParamter();
        isMarkButton->setEnabled(true);
    }
    else
    {
        isMarkButton->setEnabled(false);
    }
    this->setFocus();
}

void ControlWindow::setDrawShape(int shapeId)
{
    this->drawLable->setDrawShape(shapeId);
}

void ControlWindow::resizeEvent(QResizeEvent *e)
{
    updateExpandLeft();
    updateExpandRight();
    QWidget::resizeEvent(e);
}

void ControlWindow::closeEvent(QCloseEvent *event)
{
    if(isMark)
    {
        saveMarkDataResult();
    }
    writeMarkHistory();
    QWidget::closeEvent(event);
}

void ControlWindow::contextMenuEvent (QContextMenuEvent * event)
{
    QWidget::contextMenuEvent(event);
}

void ControlWindow::keyPressEvent(QKeyEvent *e)
{
    if(processMarkDataList.size() > 0)
    {
        if(e->key() == Qt::Key_A)
        {
            showPrevious();
        }
        else if(e->key() == Qt::Key_D)
        {
            showNext();
        }
        else if(this->markDataType == MarkDataType::VIDEO)
        {
            if(e->key() == Qt::Key_J)
            {
                previousVideo();
            }
            else if(e->key() == Qt::Key_L)
            {
                nextVideo();
            }
        }
        else if(e->key() == Qt::Key_E)
        {
            slotIsMark();
        }
    }
}

void ControlWindow::slotIsMark()
{
    if(currentIndex >= 0)
    {
        if(isMark)
        {
            saveMarkDataResult();
            this->isMark = false;
        }
        else
        {
            this->isMark = true;
        }
        updateIsMarkButton(this->isMark);
        updateDrawLabel(this->isMark);
    }
}

void ControlWindow::slotImageItem(QListWidgetItem *item)
{
    if(this->isMark)
    {
        saveMarkDataResult();
    }
    this->currentIndex = imageListWidget->row(item);
    loadMarkData(item->text());
    this->setFocus();
}

void ControlWindow::slotChangeClass(QString classText)
{
    if(this->isMark)
    {
        saveMarkImageResult();
    }
    loadMarkImage();
    this->setFocus();
}

void ControlWindow::slotShowFull()
{
    if(this->currentIndex >= 0)
    {
        if(showFullButton->text().contains(tr("全屏显示")))
        {
            drawScrollArea->setWindowFlags(Qt::Window);
            drawScrollArea->showFullScreen();
            showFullButton->setText(tr("退出全屏"));
        }
        else
        {
            drawScrollArea->setWindowFlags(Qt::SubWindow);
            drawScrollArea->showNormal();
            showFullButton->setText(tr("全屏显示"));
        }
    }
}

void ControlWindow::slotScrollArea(int keyValue)
{
    if(processMarkDataList.size() > 0)
    {
        if(keyValue == int(Qt::Key_A))
        {
            showPrevious();
        }
        else if(keyValue == int(Qt::Key_D))
        {
            showNext();
        }
        else if(this->markDataType == MarkDataType::VIDEO)
        {
            if(keyValue == int(Qt::Key_J))
            {
                previousVideo();
            }
            else if(keyValue == int(Qt::Key_L))
            {
                nextVideo();
            }
        }
        else if(keyValue == int(Qt::Key_E))
        {
            slotIsMark();
        }
    }
    if(keyValue == int(Qt::Key_Escape))
    {
        slotShowFull();
    }
}

void ControlWindow::slotManualMarkParamterChanged()
{
    initMarkClassBox();
}

void ControlWindow::showPrevious()
{
    int flag = 0;
    if(this->markDataType == MarkDataType::VIDEO)
    {
        this->videoIsTracking = false;
        if(currentFrameNumber - skipFrameNumber >= 0)
        {
            if(this->isMark)
                saveMarkImageResult();
            currentFrameNumber -= skipFrameNumber;
            flag = 1;
        }
        else
        {
            QMessageBox::information(this, tr("视频信息"), tr("到达视频第一帧！"));
        }
    }
    else if(this->markDataType == MarkDataType::IMAGE)
    {
        if(currentIndex > 0)
        {
            if(this->isMark)
                saveMarkImageResult();
            currentIndex--;
            flag = 1;
        }
    }
    if(flag == 1)
    {
        loadMarkImage();
    }
}

void ControlWindow::showNext()
{
    int flag = 0;
    if(this->markDataType == MarkDataType::VIDEO)
    {
        this->videoIsTracking = false;
        if(currentFrameNumber + skipFrameNumber < allCountFrame)
        {
            if(this->isMark)
                saveMarkImageResult();
            currentFrameNumber += skipFrameNumber;
            flag = 1;
            this->videoIsTracking = true;
        }
        else
        {
             QMessageBox::information(this, tr("视频信息"), tr("到达视频最后一帧！"));
        }

    }
    else if(this->markDataType == MarkDataType::IMAGE)
    {
        if(currentIndex < processMarkDataList.size() - 1)
        {
            if(this->isMark)
                saveMarkImageResult();
            currentIndex++;
            flag = 1;
        }
    }
    if(flag == 1)
    {
        loadMarkImage();
    }

}

void ControlWindow::nextVideo()
{
    int flag = 0;
    if(currentIndex < processMarkDataList.size() - 1)
    {
        if(this->isMark)
            saveMarkDataResult();
        currentIndex++;
        flag = 1;
    }
    if(flag == 1)
    {
        loadMarkData(processMarkDataList[currentIndex]);
    }
}

void ControlWindow::previousVideo()
{
    int flag = 0;
    if(currentIndex > 0)
    {
        if(this->isMark)
            saveMarkDataResult();
        currentIndex--;
        flag = 1;
    }
    if(flag == 1)
    {
        loadMarkData(processMarkDataList[currentIndex]);
    }
}

void ControlWindow::updateDrawLabel(bool isValue)
{
    this->drawLable->setEnabled(isValue);
}

void ControlWindow::updateIsMarkButton(bool isValue)
{
    if(!isValue)
    {
        isMarkButton->setText(tr("启用标注"));
        isMarkButton->setStyleSheet("background-color:#302F2F");
    }
    else
    {
        isMarkButton->setText(tr("禁止标注"));
        isMarkButton->setStyleSheet("background-color:#80FF00");
    }
}

void ControlWindow::updateImage()
{
    drawLable->clearObjects();
    drawLable->setNewQImage(currentImage);
}

void ControlWindow::updateListBox()
{
    imageListWidget->clear();
    for(int index = 0; index < processMarkDataList.size(); index++)
    {
        if(processDataFlagList[index] > 0)
        {
            QListWidgetItem *item = new QListWidgetItem(QIcon(":/qss_icons/style/rc/checkbox_checked_focus.png"),
                                                        processMarkDataList[index]);
            item->setData(Qt::UserRole, 1);
            imageListWidget->insertItem(index, item);
        }
        else
        {
            QListWidgetItem *item = new QListWidgetItem(QIcon(":/qss_icons/style/rc/checkbox_checked_disabled.png"),
                                                        processMarkDataList[index]);
            item->setData(Qt::UserRole, 0);
            imageListWidget->insertItem(index, item);
        }
    }
    if(currentIndex >= 0)
    {
        imageListWidget->setCurrentRow(currentIndex);
    }

}

void ControlWindow::updateMarkProcessLable()
{
    int markCount = 0;
    int allDataCount = processDataFlagList.size();
    for(int index = 0; index < allDataCount; index++)
    {
        if(processDataFlagList[index] <= 0)
        {
            markCount++;
        }
    }
    if(this->markDataType == MarkDataType::VIDEO)
    {
        this->markProcessLabel->setText(tr("标注进度:%1/%2 当前帧号:%3").arg(markCount).arg(allDataCount).arg(this->currentFrameNumber));
    }
    else if(this->markDataType == MarkDataType::IMAGE)
    {
        this->markProcessLabel->setText(tr("标注进度:%1/%2 当前索引:%3").arg(markCount).arg(allDataCount).arg(this->currentIndex));
    }
    else if(this->markDataType == MarkDataType::UNKNOWN)
    {
        this->markProcessLabel->setText(tr("标注进度:%1/%2").arg(markCount).arg(allDataCount));
    }
}

void ControlWindow::loadMarkData(const QString dataPath)
{   QString saveAnnotationsDir = this->markDataDir + "/../" + "Annotations";
    if(this->markDataType == MarkDataType::IMAGE)
    {
        loadImageData(dataPath, saveAnnotationsDir);
    }
    else if(this->markDataType == MarkDataType::VIDEO)
    {
        loadVideoData(dataPath, saveAnnotationsDir);
    }
    updateListBox();
    updateMarkProcessLable();
}

void ControlWindow::saveMarkDataResult()
{
    QDir makeDir;
    QString saveAnnotationsDir = this->markDataDir + "/../" + "Annotations";
    QList<MyObject> objects = drawLable->getObjects();
    if(objects.size() > 0 || videoResult.size() > 0)
    {
        if(!makeDir.exists(saveAnnotationsDir))
        {
            if(!makeDir.mkdir(saveAnnotationsDir))
            {
                qDebug() << "make Annotations dir fail!" << endl;
            }
        }
        if(this->markDataType == MarkDataType::IMAGE)
        {
            saveImageDataResult(saveAnnotationsDir, this->currentImagePath, objects);
        }
        else if(this->markDataType == MarkDataType::VIDEO)
        {
            saveVideoDataResult(saveAnnotationsDir, this->currentVideoPath, objects);
        }
    }
}

void ControlWindow::loadMarkImage()
{
    QString saveAnnotationsDir = this->markDataDir + "/../" + "Annotations";
    if(this->markDataType == MarkDataType::IMAGE)
    {
        currentImagePath =  processMarkDataList[currentIndex];
        loadImageData(currentImagePath, saveAnnotationsDir);
    }
    else if(this->markDataType == MarkDataType::VIDEO)
    {
        loadVideoImage();
    }
    updateListBox();
    updateMarkProcessLable();
}

void ControlWindow::saveMarkImageResult()
{
    QDir makeDir;
    QString saveAnnotationsDir = this->markDataDir + "/../" + "Annotations";
    QList<MyObject> objects = drawLable->getObjects();
    if(objects.size() > 0)
    {
        if(!makeDir.exists(saveAnnotationsDir))
        {
            if(!makeDir.mkdir(saveAnnotationsDir))
            {
                qDebug() << "make Annotations dir fail!" << endl;
            }
        }
        if(this->markDataType == MarkDataType::IMAGE)
        {
            saveImageDataResult(saveAnnotationsDir, this->currentImagePath, objects);
        }
        else if(this->markDataType == MarkDataType::VIDEO)
        {
            updateVideoResult(objects);
        }
    }
}

void ControlWindow::loadImageData(const QString imagePath, const QString saveAnnotationsDir)
{
    if(currentImage.load(imagePath))
    {
        currentImagePath = imagePath;
        updateImage();
        QFileInfo imageFileInfo(currentImagePath);
        QString readXmlPath= saveAnnotationsDir + "/" + imageFileInfo.completeBaseName() + ".xml";
        QFileInfo xmlFileInfo(readXmlPath);
        QList<MyObject> objects;
        if(xmlFileInfo.exists() && xmlProcess.readXML(readXmlPath, objects) == 0)
        {
            drawLable->setOjects(objects, classBox->currentText());
        }
    }
    else
    {
        QMessageBox::information(this, tr("加载图片"), tr("加载图片失败！"));
    }
}

void ControlWindow::saveImageDataResult(const QString &saveAnnotationsDir, const QString &imagePath, const QList<MyObject> &objects)
{
    QFileInfo imageFileInfo(imagePath);
    QString saveXmlPath = saveAnnotationsDir + "/" + imageFileInfo.completeBaseName() + ".xml";
    QFileInfo xmlFileInfo(saveXmlPath);

    QMessageBox::StandardButton result = QMessageBox::question(this, tr("保存标注信息"), tr("是否保存标注信息?"),
                                                           QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    if(result == QMessageBox::Yes)
    {
        if(objects.size() > 0)
        {
            if(xmlProcess.createXML(saveXmlPath, currentImagePath, currentImage.width(),
                                    currentImage.height(), objects) == 0)
            {
                if(currentIndex >= 0)
                {
                    processDataFlagList[currentIndex] = 0;
                }
            }
            else
            {
                QMessageBox::information(this, tr("保存XML"), tr("保存XML文件失败！"));
            }
        }
        else if(xmlFileInfo.exists())
        {
            QFile tempFile(saveXmlPath);
            tempFile.remove();
        }
    }
}

void ControlWindow::loadVideoData(const QString videoPath, const QString saveAnnotationsDir)
{
    allCountFrame = 0;
    currentFrameNumber = -1;
    videoResult.clear();
    currentVideoPath = videoPath;
    if(videoProcess.isOpen())
    {
        videoProcess.closeVideo();
    }
    this->skipFrameNumber = VideoMarkParamterConfig::getSkipFrameNumber();
    if(videoProcess.openVideo(videoPath) == 0)
    {
        QFileInfo videoFileInfo(currentVideoPath);
        QString readJsonPath= saveAnnotationsDir + "/" + videoFileInfo.completeBaseName() + ".json";
        QFileInfo jsonFileInfo(readJsonPath);
        if(jsonFileInfo.exists() && jsonProcess.readJSON(readJsonPath, videoResult, skipFrameNumber) == 0)
        {
            ;
        }
        allCountFrame = videoProcess.getFrameCount();
        currentFrameNumber = 0;
        if(videoMultipletracking != NULL)
            videoMultipletracking->initTrackingData();
        loadVideoImage();
    }
    else
    {
        QMessageBox::information(this, tr("打开视频"), tr("打开视频失败！"));
    }
}

void ControlWindow::saveVideoDataResult(const QString &saveAnnotationsDir, const QString &videoPath, const QList<MyObject> &objects)
{
    QFileInfo videoFileInfo(videoPath);
    QString saveJsonPath = saveAnnotationsDir + "/" + videoFileInfo.completeBaseName() + ".json";
    QFileInfo jsonFileInfo(saveJsonPath);
    updateVideoResult(objects);
    videoProcess.closeVideo();
    if(videoResult.size() > 0)
    {
        if(jsonProcess.createJSON(saveJsonPath, currentVideoPath, videoResult, skipFrameNumber) == 0)
        {
            if(currentIndex >= 0)
            {
                processDataFlagList[currentIndex] = 0;
            }
        }
        else
        {
            QMessageBox::information(this, tr("保存JSON"), tr("保存JSON文件失败！"));
        }

    }
    else if(jsonFileInfo.exists())
    {
        QFile tempFile(saveJsonPath);
        tempFile.remove();
    }
}

void ControlWindow::loadVideoImage()
{
    cv::Mat frame;
    cv::Mat rgbFrame;
    videoProcess.setFramePosition(currentFrameNumber);
    if(videoProcess.readFrame(frame) == 0)
    {
        cv::cvtColor(frame, rgbFrame, cv::COLOR_BGR2RGB);
        currentImage = QImage((uchar*)rgbFrame.data, rgbFrame.cols, rgbFrame.rows, QImage::Format_RGB888);
        updateImage();
        if(videoMultipletracking != NULL && this->videoIsTracking)
        {
            videoTracking(preFrame, frame);
        }
        QList<MyObject> objects = videoResult.value(currentFrameNumber, QList<MyObject>());
        drawLable->setOjects(objects, classBox->currentText());
        preFrame = frame.clone();
    }
    else
    {
        QMessageBox::information(this, tr("读取视频图像"), tr("加载视频图像失败！"));
    }
}

void ControlWindow::updateVideoResult(const QList<MyObject> &objects)
{
    QMessageBox::StandardButton result = QMessageBox::question(this, tr("更新标注信息"), tr("是否更新当前帧的标注信息?"),
                                                           QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    if(result == QMessageBox::Yes)
    {
        if(objects.size() > 0)
        {
            videoResult[currentFrameNumber] = objects;
        }
    }
}

void ControlWindow::initVideoTracking()
{
    if(videoMultipletracking != NULL)
    {
        delete videoMultipletracking;
        videoMultipletracking = NULL;
    }
    if(VideoMarkParamterConfig::getIsTracking())
    {
        videoMultipletracking = new VideoMultipletracking();
    }
}

void ControlWindow::videoTracking(const cv::Mat& preFrame, const cv::Mat& frame)
{
    if(videoMultipletracking != NULL)
    {
        QList<MyObject> preObjects = videoResult.value(currentFrameNumber - skipFrameNumber, QList<MyObject>());
        QList<MyObject> nextObjects = videoResult.value(currentFrameNumber, QList<MyObject>());
        videoMultipletracking->tracking(preFrame, frame, preObjects, nextObjects);
        nextObjects = videoMultipletracking->getTrackingResult();
        if(nextObjects.count() > 0)
        {
            videoResult[currentFrameNumber] = nextObjects;
        }
    }
}

void ControlWindow::setMarkDataParamter()
{
    if(this->markDataType == MarkDataType::VIDEO)
    {
        this->skipFrameNumber = VideoMarkParamterConfig::getSkipFrameNumber();
        if(this->skipFrameNumber < 1)
        {
            this->skipFrameNumber = 1;
        }
        initVideoTracking();
    }
}

void ControlWindow::initUI()
{
    showClass = new QLabel(tr("显示类别："));
    classBox = new QComboBox();
    initMarkClassBox();

    QHBoxLayout *showClassLayout = new QHBoxLayout();
    showClassLayout->setSpacing(10);
    showClassLayout->setAlignment(Qt::AlignCenter);
    showClassLayout->addWidget(showClass);
    showClassLayout->addWidget(classBox);

    showFullButton = new QPushButton(tr("全屏显示"));
    isMarkButton = new QPushButton(tr("启用标注"));
    isMarkButton->setStyleSheet("background-color:#302F2F");
    markProcessLabel = new QLabel(tr(""));
    updateMarkProcessLable();

    QHBoxLayout *centerTopLayout = new QHBoxLayout();
    centerTopLayout->setAlignment(Qt::AlignCenter);
    centerTopLayout->setSpacing(50);
    centerTopLayout->addLayout(showClassLayout);
    centerTopLayout->addWidget(showFullButton);
    centerTopLayout->addWidget(isMarkButton);
    centerTopLayout->addWidget(markProcessLabel);

    drawLable = new EditableLabel();
    drawScrollArea = new MyScrollArea();
    drawScrollArea->setAlignment(Qt::AlignCenter);
    drawScrollArea->setBackgroundRole(QPalette::Dark);
    drawScrollArea->setAutoFillBackground(true);
    //drawScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);  //控件大小 小于 视窗大小时，默认不会显示滚动条
    //drawScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);    //强制显示滚动条。
    drawScrollArea->setWidget(drawLable);
    drawLable->setEnabled(false);
    updateImage();

    QVBoxLayout *centerLayout = new QVBoxLayout();
    centerLayout->setSpacing(10);
    centerLayout->setMargin(0);
    centerLayout->setAlignment(Qt::AlignCenter);
    centerLayout->addLayout(centerTopLayout);
    centerLayout->addWidget(drawScrollArea);

    centerWidget = new QWidget(this);
    centerWidget->setLayout(centerLayout);

    imageListWidget = new QListWidget();

    initExpandLeft();
    initExpandRight();

    QHBoxLayout *mainLayout = new QHBoxLayout();
    //mainLayout->setMargin(10); //设置这个对话框的边距
    //mainLayout->setSpacing(10);  //设置各个控件之间的边距
    //mainLayout->addWidget(expand1);
    mainLayout->addWidget(centerWidget);
    mainLayout->addWidget(expand2);
    mainLayout->addWidget(imageListWidget);
    mainLayout->setStretchFactor(centerWidget, 4);
    mainLayout->setStretchFactor(imageListWidget, 1);

    this->setLayout(mainLayout);
    //this->setMaximumSize(700,520);
    this->setMinimumSize(1000, 600);
    this->setWindowTitle(tr("样本标注"));
}

void ControlWindow::initConnect()
{
    connect(imageListWidget, &QListWidget::itemClicked, this, &ControlWindow::slotImageItem);

    connect(showFullButton, &QPushButton::clicked, this, &ControlWindow::slotShowFull);
    connect(drawScrollArea, &MyScrollArea::signalsKey, this, &ControlWindow::slotScrollArea);
    connect(isMarkButton, &QPushButton::clicked, this, &ControlWindow::slotIsMark);
}

void ControlWindow::initImageList()
{
    processDataFlagList.clear();
    readMarkHistory();
}

void ControlWindow::initData()
{
    initMarkData(".", MarkDataType::UNKNOWN);
    initImageData();
    initVideoData();
    videoMultipletracking = NULL;
    ManualParamterConfig::loadConfig();
    VideoMarkParamterConfig::loadConfig();
}

void ControlWindow::initMarkData(const QString dirPath, const MarkDataType dataType)
{
    this->processMarkDataList.clear();
    this->processDataFlagList.clear();
    this->markDataType = dataType;
    this->markDataDir = dirPath;
    this->isMark = false;
    this->currentIndex = -1;
    this->currentImage = QImage(tr(":/images/images/play.png"));
}

void ControlWindow::initImageData()
{
    this->currentImagePath = "";
}

void ControlWindow::initVideoData()
{
    this->currentVideoPath = "";
    this->currentFrameNumber = -1;
    this->allCountFrame = 0;
    this->skipFrameNumber = 1;
    this->videoIsTracking = false;
    this->videoResult.clear();
}

void ControlWindow::initMarkClassBox()
{
    disconnect(classBox, &QComboBox::currentTextChanged, this, &ControlWindow::slotChangeClass);
    classBox->clear();
    classBox->addItem(QString("All"));
    QMap<QString, QString> markClassData = ManualParamterConfig::getMarkClass();
    for(QMap<QString, QString>::const_iterator classIterator = markClassData.constBegin();
        classIterator != markClassData.constEnd(); ++classIterator)
    {
        classBox->addItem(classIterator.key());
    }
    connect(classBox, &QComboBox::currentTextChanged, this, &ControlWindow::slotChangeClass);
}

void ControlWindow::initExpandLeft()
{
//    leftTabXxpanded1 = true;
//    leftTabminmumwidth1 = 0;
//    expand1 = new WExpand();
//    expand1->setAngle(false);

//    customAnimation1 = new CustomAnimation(leftGrounpBox, centerWidget);
//    connect(expand1, &WExpand::signalStatusChangeed,[&](bool expanded){
//        QPropertyAnimation *animation = new  QPropertyAnimation(leftGrounpBox, "maximumWidth", this);
//        if(expanded){
//            animation->setStartValue(leftGrounpBox->width());
//            animation->setEndValue(leftTabminmumwidth1);
//        }
//        else{
//            animation->setStartValue(leftGrounpBox->width());
//            animation->setEndValue(0);
//        }
//        leftTabXxpanded1 = expanded;
//        animation->setDuration(300);
//        animation->start();
//        connect(animation,&QPropertyAnimation::finished,[animation](){
//            animation->deleteLater();
//        });
//    });
}

void ControlWindow::initExpandRight()
{
    leftTabXxpanded2 = true;
    leftTabminmumwidth2 = 0;
    expand2 = new WExpand();
    expand2->setAngle(true);
    customAnimation2 = new CustomAnimation(imageListWidget, centerWidget);
    connect(expand2, &WExpand::signalStatusChangeed,[&](bool expanded){
        QPropertyAnimation *animation = new  QPropertyAnimation(imageListWidget, "maximumWidth", this);
        if(expanded){
            animation->setStartValue(imageListWidget->width());
            animation->setEndValue(leftTabminmumwidth2);
        }
        else{
            animation->setStartValue(imageListWidget->width());
            animation->setEndValue(0);
        }
        leftTabXxpanded2 = expanded;
        animation->setDuration(300);
        animation->start();
        connect(animation,&QPropertyAnimation::finished,[animation](){
            animation->deleteLater();
        });
    });
}

void ControlWindow::updateExpandLeft()
{
//    if (leftTabXxpanded1)
//    {
//        leftGrounpBox->setMaximumWidth(200);
//        leftTabminmumwidth1 = leftGrounpBox->width();
//    }
}

void ControlWindow::updateExpandRight()
{
    if(leftTabXxpanded2)
    {
        imageListWidget->setMaximumWidth(200);
        leftTabminmumwidth2 = imageListWidget->width();
    }
}

void ControlWindow::readMarkHistory()
{
    QList<QString> historyDatas;
    int historyCount = 0;
    historyDatas = historyProcess.readHistoryData(this->markDataDir);
    historyCount = historyDatas.size();
    for(int index = 0; index < processMarkDataList.size(); index++)
    {
        int loop = 0;
        for(loop = 0; loop < historyCount; loop++)
        {
            if(processMarkDataList[index].contains(historyDatas[loop]))
            {
                break;
            }
        }
        if(historyCount > 0 && loop < historyCount)
        {
            processDataFlagList.append(0);
        }
        else
        {
            processDataFlagList.append(1);
        }
    }
}

void ControlWindow::writeMarkHistory()
{
    QList<QString> tempHistoryDatas;
    tempHistoryDatas.clear();
    for(int index = 0; index < processMarkDataList.size(); index++)
    {
        if(processDataFlagList[index] <= 0)
        {
            tempHistoryDatas.append(processMarkDataList[index]);
        }
    }
    QList<QString> historyDatas = historyProcess.getFileName(tempHistoryDatas);
    historyProcess.writeHistoryData(this->markDataDir, historyDatas);
}
