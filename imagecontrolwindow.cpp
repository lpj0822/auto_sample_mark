#pragma execution_character_set("utf-8")
#include "imagecontrolwindow.h"
#include <QMessageBox>
#include <QDebug>

#include "manualparamterconfig.h"
#include "videomarkparamterconfig.h"
#include "videoparameterwindow.h"

ImageControlWindow::ImageControlWindow(QWidget *parent)
    : ControlWindow(parent)
{
    initData();
    initConnect();
}

ImageControlWindow::~ImageControlWindow()
{
    if(videoMultipletracking != NULL)
    {
        delete videoMultipletracking;
        videoMultipletracking = NULL;
    }
}

void ImageControlWindow::setMarkDataList(const QString markDataDir, const QList<QString> markDataList, const MarkDataType dataType)
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

void ImageControlWindow::slotIsMark()
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

void ImageControlWindow::slotImageItem(QListWidgetItem *item)
{
    if(this->isMark)
    {
        saveMarkDataResult();
    }
    this->currentIndex = markDataListWidget->row(item);
    loadMarkData(item->text());
    this->setFocus();
}

void ImageControlWindow::slotChangeClass(QString classText)
{
    if(this->isMark)
    {
        saveMarkImageResult();
    }
    loadMarkImage();
    this->setFocus();
}

void ImageControlWindow::slotShowFull()
{
    if(this->currentIndex >= 0)
    {
        if(showFullButton->text().contains(tr("全屏显示")))
        {
            drawMarkDataWidget->setWindowFlags(Qt::Window);
            drawMarkDataWidget->showFullScreen();
            showFullButton->setText(tr("退出全屏"));
        }
        else
        {
            drawMarkDataWidget->setWindowFlags(Qt::SubWindow);
            drawMarkDataWidget->showNormal();
            showFullButton->setText(tr("全屏显示"));
        }
    }
}

void ImageControlWindow::slotScrollArea(int keyValue)
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

void ImageControlWindow::closeEvent(QCloseEvent *event)
{
    if(isMark)
    {
        saveMarkDataResult();
    }
    writeMarkHistory();
    QWidget::closeEvent(event);
}

void ImageControlWindow::keyPressEvent(QKeyEvent *e)
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

void ImageControlWindow::showPrevious()
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

void ImageControlWindow::showNext()
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

void ImageControlWindow::nextVideo()
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

void ImageControlWindow::previousVideo()
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

void ImageControlWindow::updateDrawLabel(bool isValue)
{
    this->drawLable->setEnabled(isValue);
}

void ImageControlWindow::updateImage()
{
    drawLable->clearObjects();
    drawLable->setNewQImage(currentImage);
}

void ImageControlWindow::updateMarkProcessLable()
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

void ImageControlWindow::loadMarkData(const QString dataPath)
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

void ImageControlWindow::saveMarkDataResult()
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

void ImageControlWindow::loadMarkImage()
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

void ImageControlWindow::loadImageData(const QString imagePath, const QString saveAnnotationsDir)
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

void ImageControlWindow::saveImageDataResult(const QString &saveAnnotationsDir, const QString &imagePath, const QList<MyObject> &objects)
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

void ImageControlWindow::loadVideoData(const QString videoPath, const QString saveAnnotationsDir)
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

void ImageControlWindow::saveVideoDataResult(const QString &saveAnnotationsDir, const QString &videoPath, const QList<MyObject> &objects)
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

void ImageControlWindow::loadVideoImage()
{
    cv::Mat frame;
    cv::Mat rgbFrame;
    videoProcess.setFramePosition(currentFrameNumber);
    if(videoProcess.readFrame(frame) == 0)
    {
        cv::cvtColor(frame, rgbFrame, cv::COLOR_BGR2RGB);
        currentImage = QImage((uchar*)rgbFrame.data, rgbFrame.cols, rgbFrame.rows, QImage::Format_RGB888);
        updateImage();
        if(this->isMark && videoMultipletracking != NULL && this->videoIsTracking)
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

void ImageControlWindow::updateVideoResult(const QList<MyObject> &objects)
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

void ImageControlWindow::initVideoTracking()
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

void ImageControlWindow::videoTracking(const cv::Mat& preFrame, const cv::Mat& frame)
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

void ImageControlWindow::setMarkDataParamter()
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

void ImageControlWindow::initConnect()
{
    connect(markDataListWidget, &QListWidget::itemClicked, this, &ImageControlWindow::slotImageItem);
    connect(showFullButton, &QPushButton::clicked, this, &ImageControlWindow::slotShowFull);
    connect(drawMarkDataWidget, &MyStackedWidget::signalsKey, this, &ImageControlWindow::slotScrollArea);
    connect(isMarkButton, &QPushButton::clicked, this, &ImageControlWindow::slotIsMark);
    connect(classBox, &QComboBox::currentTextChanged, this, &ImageControlWindow::slotChangeClass);
}

void ImageControlWindow::initData()
{
    initMarkData(".", MarkDataType::UNKNOWN);
    initImageData();
    initVideoData();
    videoMultipletracking = NULL;
    ManualParamterConfig::loadConfig();
    VideoMarkParamterConfig::loadConfig();
}

void ImageControlWindow::initImageData()
{
    this->currentImagePath = "";
}

void ImageControlWindow::initVideoData()
{
    this->currentVideoPath = "";
    this->currentFrameNumber = -1;
    this->allCountFrame = 0;
    this->skipFrameNumber = 1;
    this->videoIsTracking = false;
    this->videoResult.clear();
}

void ImageControlWindow::initImageList()
{
    processDataFlagList.clear();
    readMarkHistory();
}

void ImageControlWindow::saveMarkImageResult()
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
