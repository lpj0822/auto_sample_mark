﻿#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "imagesegmentcontrolwindow.h"
#include <QMessageBox>
#include <QDebug>
#include "sampleMarkParam/segmentparamterconfig.h"

ImageSegmentControlWindow::ImageSegmentControlWindow(QWidget *parent)
    : ControlWindow(parent)
{
    initDrawWidget();
    initData();
    initConnect();
}

ImageSegmentControlWindow::~ImageSegmentControlWindow()
{

}

void ImageSegmentControlWindow::setMarkDataList(const QString markDataDir, const QList<QString> markDataList, const MarkDataType dataType)
{
    initMarkData(markDataDir, dataType);
    initImageData();

    updateIsMarkButton(this->isMark);
    updateDrawLabel(this->isMark);

    if(markDataList.size() > 0)
    {
        readClassConfig();
        this->processMarkDataList = markDataList;
        initImageList();
        updateListBox();
        updateMarkProcessLable();
        isMarkButton->setEnabled(true);
    }
    else
    {
        isMarkButton->setEnabled(false);
    }
    this->setFocus();
}

void ImageSegmentControlWindow::saveMarkDataList()
{
    if(this->isMark)
    {
        saveMarkDataResult();
    }
    writeMarkHistory();
}

void ImageSegmentControlWindow::setDrawShape(int shapeId)
{
    this->drawLable->setDrawShape(shapeId);
}

void ImageSegmentControlWindow::slotImageItem(QListWidgetItem *item)
{
    if(this->isMark)
    {
        saveMarkDataResult();
    }
    this->currentIndex = markDataListWidget->row(item);
    loadMarkData(item->text());
    this->setFocus();
}

void ImageSegmentControlWindow::slotChangeClass(QString classText)
{
    if(this->isMark)
    {
        saveMarkDataResult();
    }
    loadMarkImage();
    this->setFocus();
}

void ImageSegmentControlWindow::slotScrollArea(int keyValue)
{
    if(processMarkDataList.size() > 0)
    {
        switch (keyValue)
        {
        case Qt::Key_A:
            showPrevious();
            break;
        case Qt::Key_D:
            showNext();
            break;
        case Qt::Key_E:
            slotIsMark();
            break;
        }
        if(keyValue == int(Qt::ControlModifier + Qt::Key_Z))
        {
            if(isMark)
            {
                drawLable->undoDrawShape();
            }
        }
    }
    if(keyValue == int(Qt::Key_Escape))
    {
        slotShowFull();
    }
}

void ImageSegmentControlWindow::closeEvent(QCloseEvent *event)
{
    saveMarkDataList();
    ControlWindow::closeEvent(event);
}

void ImageSegmentControlWindow::keyPressEvent(QKeyEvent *event)
{
    if(processMarkDataList.size() > 0)
    {
        switch (event->key())
        {
        case Qt::Key_A:
            showPrevious();
            break;
        case Qt::Key_D:
            showNext();
            break;
        case Qt::Key_E:
            slotIsMark();
            break;
        }
    }
    if(event->modifiers() == Qt::ControlModifier)
    {
        if(event->key() == Qt::Key_M)
        {
            this->setWindowState(Qt::WindowMaximized);
        }
        else if(event->key() == Qt::Key_Z && isMark)
        {
            drawLable->undoDrawShape();
        }
    }
}

void ImageSegmentControlWindow::isMarkData()
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

void ImageSegmentControlWindow::resetDraw()
{
    drawLable->resetDraw();
}

void ImageSegmentControlWindow::showPrevious()
{
    if(this->markDataType == MarkDataType::SEGMENT)
    {
        if(currentIndex > 0)
        {
            if(this->isMark)
                saveMarkDataResult();
            currentIndex--;
             loadMarkImage();
        }
    }
}

void ImageSegmentControlWindow::showNext()
{
    if(currentIndex < processMarkDataList.size() - 1)
    {
        if(this->isMark)
            saveMarkDataResult();
        currentIndex++;
        loadMarkImage();
    }
}

void ImageSegmentControlWindow::updateDrawLabel(bool isValue)
{
    this->drawLable->setEnabled(isValue);
}

void ImageSegmentControlWindow::updateImage()
{
    drawLable->clearDraw();
    drawLable->setNewQImage(currentImage);
}

void ImageSegmentControlWindow::loadMarkData(const QString dataPath)
{
    QString saveAnnotationsDir = this->markDataDir + "/../" + "Annotations";
    QString saveSegmentLabelDir = this->markDataDir + "/../" + "SegmentLabel";
    if(this->markDataType == MarkDataType::SEGMENT)
    {
        loadImageData(dataPath, saveSegmentLabelDir, saveAnnotationsDir);
    }
    updateListBox();
    updateMarkProcessLable();
}

void ImageSegmentControlWindow::saveMarkDataResult()
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
        if(this->markDataType == MarkDataType::SEGMENT)
        {
            saveImageDataResult(saveAnnotationsDir, this->currentImagePath, objects);
        }
    }
    else
    {
        if(this->markDataType == MarkDataType::SEGMENT)
        {
            saveImageDataResult(saveAnnotationsDir, this->currentImagePath, objects);
        }
    }
}

void ImageSegmentControlWindow::loadMarkImage()
{
    QString saveSegmentLabelDir = this->markDataDir + "/../" + "SegmentLabel";
    QString saveAnnotationsDir = this->markDataDir + "/../" + "Annotations";
    if(this->markDataType == MarkDataType::SEGMENT && processMarkDataList.size() > 0)
    {
        currentImagePath =  processMarkDataList[currentIndex];
        loadImageData(currentImagePath, saveSegmentLabelDir, saveAnnotationsDir);
    }
    updateListBox();
    updateMarkProcessLable();
}

void ImageSegmentControlWindow::initDrawWidget()
{
    drawLable = new SegmentLabel();
    drawLableScrollArea = new QScrollArea();
    drawLableScrollArea->setAlignment(Qt::AlignCenter);
    drawLableScrollArea->setBackgroundRole(QPalette::Dark);
    drawLableScrollArea->setAutoFillBackground(true);
    //drawScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);  //控件大小 小于 视窗大小时，默认不会显示滚动条
    //drawScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);    //强制显示滚动条。
    drawLableScrollArea->setWidget(drawLable);
    drawMarkDataWidget->addWidget(drawLableScrollArea);

    drawLable->clearDraw();
    drawLable->setNewQImage(currentImage);
    drawLable->setEnabled(false);
    drawMarkDataWidget->setCurrentIndex(1);
}

void ImageSegmentControlWindow::initData()
{
    initMarkData(".", MarkDataType::SEGMENT);
    initImageData();
    SegmentParamterConfig::loadConfig();
}

void ImageSegmentControlWindow::initConnect()
{
    connect(markDataListWidget, &QListWidget::itemClicked, this, &ImageSegmentControlWindow::slotImageItem);
    connect(drawMarkDataWidget, &MyStackedWidget::signalsKey, this, &ImageSegmentControlWindow::slotScrollArea);
    connect(classBox, &QComboBox::currentTextChanged, this, &ImageSegmentControlWindow::slotChangeClass);
}

void ImageSegmentControlWindow::initImageList()
{
    processDataFlagList.clear();
    readMarkHistory();
}

void ImageSegmentControlWindow::loadImageData(const QString &imagePath, const QString &saveSegmentLabelDir,
                                              const QString &saveAnnotationsDir)
{
    if(currentImage.load(imagePath))
    {
        currentImagePath = imagePath;
        updateImage();
        QFileInfo imageFileInfo(currentImagePath);
        QString readSegmentPath = saveSegmentLabelDir + "/" + imageFileInfo.completeBaseName() + ".png";
        QFileInfo segmentFileInfo(readSegmentPath);
        MyObject maskObject;
        if(segmentFileInfo.exists() && maskProcess.readSegmentMask(readSegmentPath, maskObject) == 0)
        {

        }
        QString readJsonPath= saveAnnotationsDir + "/" + imageFileInfo.completeBaseName() + ".json";
        QFileInfo jsonFileInfo(readJsonPath);
        QList<MyObject> objects;
        if(jsonFileInfo.exists() && jsonProcess.readJSON(readJsonPath, objects) == 0)
        {

        }
        drawLable->setOjects(objects, classBox->currentText());
        drawLable->setMaskOject(maskObject);
    }
    else
    {
        QMessageBox::information(this, tr("加载图片"), tr("加载图片失败！"));
    }
}

void ImageSegmentControlWindow::saveImageDataResult(const QString &saveAnnotationsDir, const QString &imagePath,
                                             const QList<MyObject> &objects)
{
    QFileInfo imageFileInfo(imagePath);
    QString saveJsonPath = saveAnnotationsDir + "/" + imageFileInfo.completeBaseName() + ".json";
    QFileInfo jsonFileInfo(saveJsonPath);

    QMessageBox::StandardButton result = QMessageBox::question(this, tr("保存标注信息"), tr("是否保存标注信息?"),
                                                           QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    if(result == QMessageBox::Yes)
    {
        if(objects.size() > 0)
        {
            if(jsonProcess.createJSON(saveJsonPath, currentImagePath, currentImage.width(),
                                      currentImage.height(), objects) == 0)
            {
                if(currentIndex >= 0)
                {
                    processDataFlagList[currentIndex] = 0;
                }
            }
            else
            {
                QMessageBox::information(this, tr("保存Json"), tr("保存Json文件失败！"));
            }
        }
        else if(jsonFileInfo.exists())
        {
            QFile tempFile(saveJsonPath);
            tempFile.remove();
        }
    }
}

//void ImageSegmentControlWindow::saveImageSegmentResult(const QString &saveAnnotationsDir, const QString &imagePath,
//                                                const MyObject &maskObject)
//{
//    QFileInfo imageFileInfo(imagePath);
//    QString saveImagePath = saveAnnotationsDir + "/" + imageFileInfo.fileName();
//    QFileInfo saveFileInfo(saveImagePath);
//    QImage result = maskObject.getSegmentImage();
//    if(!result.isNull())
//    {
//        if(!result.save(saveImagePath))
//        {
//            QMessageBox::information(this, tr("保存Segment图象"), tr("保存Segment图象%1失败！").arg(saveImagePath));
//        }
//    }
//    else if(saveFileInfo.exists())
//    {
//        QFile tempFile(saveImagePath);
//        tempFile.remove();
//    }
//}

void ImageSegmentControlWindow::initImageData()
{
    this->currentImagePath = "";
}

