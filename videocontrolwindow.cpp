#pragma execution_character_set("utf-8")
#include "videocontrolwindow.h"
#include <QMessageBox>
#include <QDebug>

#include "sampleMarkParam/videomarkparamterconfig.h"

VideoControlWindow::VideoControlWindow(QWidget *parent)
    : ImageControlWindow(parent)
{
    initData();
}

VideoControlWindow::~VideoControlWindow()
{
    if(videoMultipletracking != NULL)
    {
        delete videoMultipletracking;
        videoMultipletracking = NULL;
    }
}

void VideoControlWindow::setMarkDataList(const QString markDataDir, const QList<QString> markDataList, const MarkDataType dataType)
{
    readClassConfig(markDataDir);
    initMarkData(markDataDir, dataType);
    initVideoData();
    updateIsMarkButton(this->isMark);
    updateDrawLabel(this->isMark);

    if(markDataList.size() > 0)
    {
        this->processMarkDataList = markDataList;
        initImageList();
        updateListBox();
        updateMarkProcessLable();
        setVideoMarkDataParamter();
        isMarkButton->setEnabled(true);
    }
    else
    {
        isMarkButton->setEnabled(false);
    }
    this->setFocus();
}

void VideoControlWindow::slotScrollArea(int keyValue)
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
        if(keyValue == int(Qt::Key_J))
        {
            previousVideo();
        }
        else if(keyValue == int(Qt::Key_L))
        {
            nextVideo();
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

void VideoControlWindow::keyPressEvent(QKeyEvent *e)
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
        if(e->key() == Qt::Key_J)
        {
            previousVideo();
        }
        else if(e->key() == Qt::Key_L)
        {
            nextVideo();
        }
        else if(e->key() == Qt::Key_E)
        {
            slotIsMark();
        }
    }
}

void VideoControlWindow::showPrevious()
{
    this->videoIsTracking = false;
    if(currentFrameNumber - skipFrameNumber >= 0)
    {
        if(this->isMark)
            saveMarkImageResult();
        currentFrameNumber -= skipFrameNumber;
        loadMarkImage();
    }
    else
    {
        QMessageBox::information(this, tr("视频信息"), tr("到达视频第一帧！"));
    }
}

void VideoControlWindow::showNext()
{
    this->videoIsTracking = false;
    if(currentFrameNumber + skipFrameNumber < allCountFrame)
    {
        if(this->isMark)
            saveMarkImageResult();
        currentFrameNumber += skipFrameNumber;
        this->videoIsTracking = true;
        loadMarkImage();
    }
    else
    {
        QMessageBox::information(this, tr("视频信息"), tr("到达视频最后一帧！"));
    }
}

void VideoControlWindow::nextVideo()
{
    if(currentIndex < processMarkDataList.size() - 1)
    {
        if(this->isMark)
            saveMarkDataResult();
        currentIndex++;
        loadMarkData(processMarkDataList[currentIndex]);
    }
}

void VideoControlWindow::previousVideo()
{
    if(currentIndex > 0)
    {
        if(this->isMark)
            saveMarkDataResult();
        currentIndex--;
        loadMarkData(processMarkDataList[currentIndex]);
    }
}

void VideoControlWindow::updateLabelText(int markCount)
{
    if(this->markDataType == MarkDataType::VIDEO)
    {
        this->markProcessLabel->setText(tr("标注进度:%1/%2 当前帧号:%3").arg(markCount).arg(processDataFlagList.size()
                                                                                    ).arg(this->currentFrameNumber));
    }
}

void VideoControlWindow::loadMarkData(const QString dataPath)
{   QString saveAnnotationsDir = this->markDataDir + "/../" + "Annotations";
    if(this->markDataType == MarkDataType::VIDEO)
    {
        loadVideoData(dataPath, saveAnnotationsDir);
    }
    updateListBox();
    updateMarkProcessLable();
}

void VideoControlWindow::saveMarkDataResult()
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
        if(this->markDataType == MarkDataType::VIDEO)
        {
            saveVideoDataResult(saveAnnotationsDir, this->currentVideoPath, objects);
        }
    }
}

void VideoControlWindow::loadMarkImage()
{
    if(this->markDataType == MarkDataType::VIDEO && currentIndex >= 0)
    {
        loadVideoImage();
    }
    updateListBox();
    updateMarkProcessLable();
}

void VideoControlWindow::saveMarkImageResult()
{
    QList<MyObject> objects = drawLable->getObjects();
    if(objects.size() > 0)
    {
        if(this->markDataType == MarkDataType::VIDEO)
        {
            updateVideoResult(objects);
        }
    }
}

void VideoControlWindow::loadVideoData(const QString videoPath, const QString saveAnnotationsDir)
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
        if(jsonFileInfo.exists() && jsonProcessVideo.readJSON(readJsonPath, videoResult, skipFrameNumber) == 0)
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

void VideoControlWindow::saveVideoDataResult(const QString &saveAnnotationsDir, const QString &videoPath, const QList<MyObject> &objects)
{
    QFileInfo videoFileInfo(videoPath);
    QString saveJsonPath = saveAnnotationsDir + "/" + videoFileInfo.completeBaseName() + ".json";
    QFileInfo jsonFileInfo(saveJsonPath);
    updateVideoResult(objects);
    videoProcess.closeVideo();
    if(videoResult.size() > 0)
    {
        if(jsonProcessVideo.createJSON(saveJsonPath, currentVideoPath, videoResult, skipFrameNumber) == 0)
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

void VideoControlWindow::loadVideoImage()
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

void VideoControlWindow::updateVideoResult(const QList<MyObject> &objects)
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

void VideoControlWindow::initVideoTracking()
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

void VideoControlWindow::videoTracking(const cv::Mat& preFrame, const cv::Mat& frame)
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

void VideoControlWindow::setVideoMarkDataParamter()
{
    this->skipFrameNumber = VideoMarkParamterConfig::getSkipFrameNumber();
    if(this->skipFrameNumber < 1)
    {
        this->skipFrameNumber = 1;
    }
    initVideoTracking();
}

void VideoControlWindow::initData()
{
    initMarkData(".", MarkDataType::VIDEO);
    initVideoData();
    videoMultipletracking = NULL;
    VideoMarkParamterConfig::loadConfig();
}

void VideoControlWindow::initVideoData()
{
    this->currentVideoPath = "";
    this->currentFrameNumber = -1;
    this->allCountFrame = 0;
    this->skipFrameNumber = 1;
    this->videoIsTracking = false;
    this->videoResult.clear();
}
