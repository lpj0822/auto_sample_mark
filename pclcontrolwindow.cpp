﻿#pragma execution_character_set("utf-8")
#include "pclcontrolwindow.h"
#include <QFileInfo>
#include <QMessageBox>
#include <QDebug>
#include"sampleMarkParam/pointcloudparamterconfig.h"

PCLControlWindow::PCLControlWindow(QWidget *parent)
    : ControlWindow(parent)
{
    initData();
    initConnect();
    initDrawWidget();
}

PCLControlWindow::~PCLControlWindow()
{

}

void PCLControlWindow::setMarkDataList(const QString markDataDir, const QList<QString> markDataList, const MarkDataType dataType)
{
    initMarkData(markDataDir, dataType);

    updateIsMarkButton(this->isMark);
    drawPointCloud->setIsSelect(this->isMark);

    if(markDataList.size() > 0)
    {
        this->processMarkDataList = markDataList;
        readMarkHistory();
        updateListBox();
        isMarkButton->setEnabled(true);
    }
    else
    {
        isMarkButton->setEnabled(false);
    }
    this->setFocus();
}

void PCLControlWindow::slotIsMark()
{
    if(currentIndex >= 0)
    {
        if(isMark)
        {
            this->isMark = false;
        }
        else
        {
            this->isMark = true;
        }
        updateIsMarkButton(this->isMark);
        drawPointCloud->setIsSelect(this->isMark);
    }
}

void PCLControlWindow::slotImageItem(QListWidgetItem *item)
{
    this->currentIndex = markDataListWidget->row(item);
    loadPointCloudData(item->text());
    this->setFocus();
}

void PCLControlWindow::slotScrollArea(int keyValue)
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
    }
    if(keyValue == int(Qt::Key_Escape))
    {
        slotShowFull();
    }
}

void PCLControlWindow::closeEvent(QCloseEvent *event)
{
    QWidget::closeEvent(event);
}

void PCLControlWindow::keyPressEvent(QKeyEvent *e)
{
    if(processMarkDataList.size() > 0)
    {
        switch (e->key())
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
}

void PCLControlWindow::showPrevious()
{
    if(currentIndex > 0)
    {
        currentIndex--;
        loadMarkPointCloud();
    }
}

void PCLControlWindow::showNext()
{
    if(currentIndex < processMarkDataList.size() - 1)
    {
        currentIndex++;
        loadMarkPointCloud();
    }
}

void PCLControlWindow::loadMarkPointCloud()
{
    if(this->markDataType == MarkDataType::PCD && processMarkDataList.size() > 0)
    {
        currentPCDPath =  processMarkDataList[currentIndex];
        loadPointCloudData(currentPCDPath);
    }
    updateListBox();
}

void PCLControlWindow::loadPointCloudData(const QString pcdFilePath)
{
    drawPointCloud->clearPoints();
    if(drawPointCloud->setNewPointCloud(pcdFilePath) != -1)
    {
        currentPCDPath = pcdFilePath;
        QFileInfo pcdFileInfo(currentPCDPath);
        QString saveAnnotationsDir = this->markDataDir + "/../" + "Annotations";
        QString readJsonPath= saveAnnotationsDir + "/" + pcdFileInfo.completeBaseName() + ".json";
        qDebug() << readJsonPath << endl;
        QFileInfo jsonFileInfo(readJsonPath);
        QList<MyObject> objects;
        if(jsonFileInfo.exists() && jsonProcess.readJSON(readJsonPath, objects) == 0)
        {
            drawPointCloud->setOjects(objects, classBox->currentText());
        }
    }
    else
    {
        QMessageBox::information(this, tr("加载pcd"), tr("加载pcd失败！"));
    }
    updateListBox();
}

void PCLControlWindow::initDrawWidget()
{
    drawPointCloud = new PCLViewer(this);
    drawMarkDataWidget->addWidget(drawPointCloud);
    drawMarkDataWidget->setCurrentIndex(1);
}

void PCLControlWindow::initData()
{
    initMarkData(".", MarkDataType::PCD);
    currentPCDPath = "";
    PointCloudParamterConfig::loadConfig();
}

void PCLControlWindow::initConnect()
{
    connect(markDataListWidget, &QListWidget::itemClicked, this, &PCLControlWindow::slotImageItem);
    connect(drawMarkDataWidget, &MyStackedWidget::signalsKey, this, &PCLControlWindow::slotScrollArea);
    connect(isMarkButton, &QPushButton::clicked, this, &PCLControlWindow::slotIsMark);
}
