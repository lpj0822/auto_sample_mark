﻿#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "imagecontrolwindow.h"
#include <QMessageBox>
#include <QDebug>

ImageControlWindow::ImageControlWindow(QWidget *parent)
    : ControlWindow(parent)
{
    initDrawWidget();
    initData();
    initConnect();
}

ImageControlWindow::~ImageControlWindow()
{

}

void ImageControlWindow::setMarkDataList(const QString markDataDir, const QList<QString> markDataList, const MarkDataType dataType)
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

void ImageControlWindow::saveMarkDataList()
{
    if(this->isMark)
    {
        saveMarkDataResult();
    }
    writeMarkHistory();
}

void ImageControlWindow::setDrawShape(int shapeId)
{
    this->drawLable->setDrawShape(shapeId);
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
        saveMarkDataResult();
    }
    loadMarkImage();
    this->setFocus();
}

void ImageControlWindow::slotScrollArea(int keyValue)
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

void ImageControlWindow::closeEvent(QCloseEvent *event)
{
    saveMarkDataList();
    ControlWindow::closeEvent(event);
}

void ImageControlWindow::keyPressEvent(QKeyEvent *event)
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

void ImageControlWindow::isMarkData()
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

void ImageControlWindow::resetDraw()
{
    drawLable->resetDraw();
}

void ImageControlWindow::showPrevious()
{
    if(this->markDataType == MarkDataType::IMAGE)
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

void ImageControlWindow::showNext()
{
    if(currentIndex < processMarkDataList.size() - 1)
    {
        if(this->isMark)
            saveMarkDataResult();
        currentIndex++;
        loadMarkImage();
    }
}

void ImageControlWindow::updateDrawLabel(bool isValue)
{
    this->drawLable->setEnabled(isValue);
}

void ImageControlWindow::updateImage()
{
    drawLable->clearDraw();
    drawLable->setNewQImage(currentImage);
}

void ImageControlWindow::loadMarkData(const QString dataPath)
{   QString saveAnnotationsDir = this->markDataDir + "/../" + "Annotations";
    if(this->markDataType == MarkDataType::IMAGE)
    {
        loadImageData(dataPath, saveAnnotationsDir);
    }
    updateListBox();
    updateMarkProcessLable();
}

void ImageControlWindow::saveMarkDataResult()
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
    }
}

void ImageControlWindow::loadMarkImage()
{
    QString saveAnnotationsDir = this->markDataDir + "/../" + "Annotations";
    if(this->markDataType == MarkDataType::IMAGE && processMarkDataList.size() > 0)
    {
        currentImagePath =  processMarkDataList[currentIndex];
        loadImageData(currentImagePath, saveAnnotationsDir);
    }
    updateListBox();
    updateMarkProcessLable();
}

void ImageControlWindow::initDrawWidget()
{
    drawLable = new EditableLabel();
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

void ImageControlWindow::initData()
{
    initMarkData(".", MarkDataType::IMAGE);
    initImageData();
}

void ImageControlWindow::initConnect()
{
    connect(markDataListWidget, &QListWidget::itemClicked, this, &ImageControlWindow::slotImageItem);
    connect(drawMarkDataWidget, &MyStackedWidget::signalsKey, this, &ImageControlWindow::slotScrollArea);
    connect(classBox, &QComboBox::currentTextChanged, this, &ImageControlWindow::slotChangeClass);
}

void ImageControlWindow::initImageList()
{
    processDataFlagList.clear();
    readMarkHistory();
}

void ImageControlWindow::loadImageData(const QString imagePath, const QString saveAnnotationsDir)
{
    if(currentImage.load(imagePath))
    {
        currentImagePath = imagePath;
        updateImage();
        QFileInfo imageFileInfo(currentImagePath);
        QString readJsonPath= saveAnnotationsDir + "/" + imageFileInfo.completeBaseName() + ".json";
        QFileInfo jsonFileInfo(readJsonPath);
        QList<MyObject> objects;
        if(jsonFileInfo.exists() && jsonProcess.readJSON(readJsonPath, objects) == 0)
        {
            drawLable->setOjects(objects, classBox->currentText());
        }
//        currentImagePath = imagePath;
//        updateImage();
//        QFileInfo imageFileInfo(currentImagePath);
//        QString readXmlPath= saveAnnotationsDir + "/" + imageFileInfo.completeBaseName() + ".xml";
//        QFileInfo xmlFileInfo(readXmlPath);
//        QList<MyObject> objects;
//        if(xmlFileInfo.exists() && xmlProcess.readXML(readXmlPath, objects) == 0)
//        {
//            drawLable->setOjects(objects, classBox->currentText());
//        }
    }
    else
    {
        QMessageBox::information(this, tr("加载图片"), tr("该图片加载失败,请点击下一张图片！"));
    }
}

void ImageControlWindow::saveImageDataResult(const QString &saveAnnotationsDir, const QString &imagePath,
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
//    QFileInfo imageFileInfo(imagePath);
//    QString saveXmlPath = saveAnnotationsDir + "/" + imageFileInfo.completeBaseName() + ".xml";
//    QFileInfo xmlFileInfo(saveXmlPath);

//    QMessageBox::StandardButton result = QMessageBox::question(this, tr("保存标注信息"), tr("是否保存标注信息?"),
//                                                           QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
//    if(result == QMessageBox::Yes)
//    {
//        if(objects.size() > 0)
//        {
//            if(xmlProcess.createXML(saveXmlPath, currentImagePath, currentImage.width(),
//                                    currentImage.height(), objects) == 0)
//            {
//                if(currentIndex >= 0)
//                {
//                    processDataFlagList[currentIndex] = 0;
//                }
//            }
//            else
//            {
//                QMessageBox::information(this, tr("保存XML"), tr("保存XML文件失败！"));
//            }
//        }
//        else if(xmlFileInfo.exists())
//        {
//            QFile tempFile(saveXmlPath);
//            tempFile.remove();
//        }
//    }
}

void ImageControlWindow::initImageData()
{
    this->currentImagePath = "";
}

