#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
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

#include "sampleMarkParam/manualparamterconfig.h"

ControlWindow::ControlWindow(QWidget *parent)
    : QWidget(parent)
{
    init();
    initUI();
    connect(showFullButton, &QPushButton::clicked, this, &ControlWindow::slotShowFull);
    connect(isMarkButton, &QPushButton::clicked, this, &ControlWindow::slotIsMark);
    connect(resetButton, &QPushButton::clicked, this, &ControlWindow::slotReset);
}

ControlWindow::~ControlWindow()
{
    
}

void ControlWindow::setMarkDataList(const QString markDataDir, const QList<QString> markDataList, const MarkDataType dataType)
{
    initMarkData(markDataDir, dataType);
    this->processMarkDataList = markDataList;
}

void ControlWindow::saveMarkDataList()
{
    writeMarkHistory();
}

void ControlWindow::setDrawShape(int shapeId)
{
    qDebug() << "shape:" << shapeId;
}

void ControlWindow::readClassConfig()
{
    QDir tempDir(this->markDataDir);
    if(!tempDir.exists())
    {
        return;
    }
    QString saveClassPath = this->markDataDir + "/../" + "class.json";
    switch(markDataType)
    {
    case MarkDataType::IMAGE:
    case MarkDataType::VIDEO:
    {
        if(ManualParamterConfig::loadDetClassConfig(saveClassPath) == 0)
        {
            initMarkClassBox();
        }
        break;
    }
    case MarkDataType::SEGMENT:
    {
        if(ManualParamterConfig::loadSegClassConfig(saveClassPath) == 0)
        {
            initMarkClassBox();
        }
        break;
    }
    default:
        break;
    }
}

void ControlWindow::saveClassConfig()
{
    QDir tempDir(this->markDataDir);
    if(!tempDir.exists())
    {
        return;
    }
    QString saveClassPath = this->markDataDir + "/../" + "class.json";
    switch(markDataType)
    {
    case MarkDataType::IMAGE:
    case MarkDataType::VIDEO:
    {
        if(ManualParamterConfig::saveDetClassConfig(saveClassPath) == 0)
        {
            ManualParamterConfig::saveConfig();
        }
        break;
    }
    case MarkDataType::SEGMENT:
    {
        if(ManualParamterConfig::saveSegClassConfig(saveClassPath) == 0)
        {
            ManualParamterConfig::saveConfig();
        }
        break;
    }
    default:
        break;
    }
}

void ControlWindow::resizeEvent(QResizeEvent *e)
{
    updateExpandLeft();
    updateExpandRight();
    QWidget::resizeEvent(e);
}

void ControlWindow::contextMenuEvent (QContextMenuEvent * event)
{
    QWidget::contextMenuEvent(event);
}

void ControlWindow::slotManualMarkParamterChanged()
{
    initMarkClassBox();
}

void ControlWindow::slotShowFull()
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

void ControlWindow::slotIsMark()
{
    isMarkData();
}

void ControlWindow::slotReset()
{
    resetDraw();
}


void ControlWindow::resetDraw()
{

}

void ControlWindow::isMarkData()
{

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

void ControlWindow::updateListBox()
{
    markDataListWidget->clear();
    for(int index = 0; index < processMarkDataList.size(); index++)
    {
        if(processDataFlagList[index] > 0)
        {
            QListWidgetItem *item = new QListWidgetItem(QIcon(":/qss_icons/style/rc/checkbox_checked_focus.png"),
                                                        processMarkDataList[index]);
            item->setData(Qt::UserRole, 1);
            markDataListWidget->insertItem(index, item);
        }
        else
        {
            QListWidgetItem *item = new QListWidgetItem(QIcon(":/qss_icons/style/rc/checkbox_checked_disabled.png"),
                                                        processMarkDataList[index]);
            item->setData(Qt::UserRole, 0);
            markDataListWidget->insertItem(index, item);
        }
    }
    if(currentIndex >= 0)
    {
        markDataListWidget->setCurrentRow(currentIndex);
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
    updateLabelText(markCount);
}

void ControlWindow::updateLabelText(int markCount)
{
    this->markProcessLabel->setText(tr("标注进度:%1/%2 当前索引:%3").arg(markCount).arg(processDataFlagList.size()
                                                                                ).arg(this->currentIndex));
}

void ControlWindow::init()
{
    initMarkData("/temp", MarkDataType::UNKNOWN);
    ManualParamterConfig::loadConfig();
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
    resetButton = new QPushButton(tr("恢复原位"));
    isMarkButton->setStyleSheet("background-color:#302F2F");
    markProcessLabel = new QLabel(tr(""));

    QHBoxLayout *centerTopLayout = new QHBoxLayout();
    centerTopLayout->setAlignment(Qt::AlignCenter);
    centerTopLayout->setSpacing(50);
    centerTopLayout->addLayout(showClassLayout);
    centerTopLayout->addWidget(showFullButton);
    centerTopLayout->addWidget(isMarkButton);
    centerTopLayout->addWidget(resetButton);
    centerTopLayout->addWidget(markProcessLabel);

    drawMarkDataWidget = new MyStackedWidget(this);
    baseLabel = new QLabel();
    baseLabel->setPixmap(QPixmap::fromImage(currentImage));
    lableScrollArea = new QScrollArea();
    lableScrollArea->setAlignment(Qt::AlignCenter);
    lableScrollArea->setBackgroundRole(QPalette::Dark);
    lableScrollArea->setAutoFillBackground(true);
    //lableScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);  //控件大小 小于 视窗大小时，默认不会显示滚动条
    //lableScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);    //强制显示滚动条。
    lableScrollArea->setWidget(baseLabel);
    drawMarkDataWidget->addWidget(lableScrollArea);
    drawMarkDataWidget->setCurrentIndex(0);

    QVBoxLayout *centerLayout = new QVBoxLayout();
    centerLayout->setSpacing(10);
    centerLayout->setMargin(0);
    centerLayout->setAlignment(Qt::AlignCenter);
    centerLayout->addLayout(centerTopLayout);
    centerLayout->addWidget(drawMarkDataWidget);

    centerWidget = new QWidget(this);
    centerWidget->setLayout(centerLayout);

    markDataListWidget = new QListWidget();

    initExpandLeft();
    initExpandRight();

    QHBoxLayout *mainLayout = new QHBoxLayout();
    //mainLayout->setMargin(10); //设置这个对话框的边距
    //mainLayout->setSpacing(10);  //设置各个控件之间的边距
    //mainLayout->addWidget(expand1);
    mainLayout->addWidget(centerWidget);
    mainLayout->addWidget(expand2);
    mainLayout->addWidget(markDataListWidget);
    mainLayout->setStretchFactor(centerWidget, 4);
    mainLayout->setStretchFactor(markDataListWidget, 1);

    this->setLayout(mainLayout);
    //this->setMaximumSize(700,520);
    this->setMinimumSize(1000, 600);
    this->setWindowTitle(tr("样本标注"));
}

void ControlWindow::initMarkData(const QString dirPath, const MarkDataType dataType)
{
    processMarkDataList.clear();
    processDataFlagList.clear();
    this->markDataType = dataType;
    this->markDataDir = dirPath;
    this->isMark = false;
    this->currentIndex = -1;
    this->currentImage = QImage(tr(":/images/images/play.png"));
}

void ControlWindow::initMarkClassBox()
{
    classBox->clear();
    classBox->addItem(QString("All"));
    QMap<QString, QString> markClassData = ManualParamterConfig::getMarkClass();
    for(QMap<QString, QString>::const_iterator classIterator = markClassData.constBegin();
        classIterator != markClassData.constEnd(); ++classIterator)
    {
        classBox->addItem(classIterator.key());
    }
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
    customAnimation2 = new CustomAnimation(markDataListWidget, centerWidget);
    connect(expand2, &WExpand::signalStatusChangeed,[&](bool expanded){
        QPropertyAnimation *animation = new  QPropertyAnimation(markDataListWidget, "maximumWidth", this);
        if(expanded){
            animation->setStartValue(markDataListWidget->width());
            animation->setEndValue(leftTabminmumwidth2);
        }
        else{
            animation->setStartValue(markDataListWidget->width());
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
        markDataListWidget->setMaximumWidth(200);
        leftTabminmumwidth2 = markDataListWidget->width();
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
