#pragma execution_character_set("utf-8")
#include "segmentlabel.h"
#include <QMenu>
#include <QMessageBox>
#include <QDebug>
#include "sampleMarkParam/manualparamterconfig.h"

SegmentLabel::SegmentLabel(QWidget *parent):
    QLabel(parent)
{
    initData();
    initConnect();
}

SegmentLabel::~SegmentLabel()
{
    if(maskImage != NULL)
    {
        delete maskImage;
        maskImage = NULL;
    }
}

void SegmentLabel::wheelEvent(QWheelEvent * event)
{
    int value = event->delta();
    float scaleRatio = value / (8 * 360.0f);
    scale += scaleRatio;
    if(scale > 2.0f)
    {
        scale = 2.0f;
    }
    else if(scale < 0.5f)
    {
        scale = 0.5f;
    }
    drawPixmap();
    QLabel::wheelEvent(event);
}

void SegmentLabel::paintEvent(QPaintEvent *e)
{
    QLabel::paintEvent(e);
    QPainter painter(this);
    int width = static_cast<int>(tempPixmap.width() * scale);
    int height = static_cast<int>(tempPixmap.height() * scale);
    QPixmap temp = tempPixmap.scaled(width, height, Qt::KeepAspectRatio);
    painter.drawPixmap(QPoint(0,0), temp);
    painter.end();
    this->resize(temp.width(), temp.height());
}

void SegmentLabel::clearObjects()
{
    if(maskImage != NULL)
    {
        delete maskImage;
        maskImage = NULL;
    }
    this->scale = 1.0f;
    drawPixmap();
}

void SegmentLabel::setNewQImage(QImage &image)
{
    mp = QPixmap::fromImage(image);
    drawPixmap();
}

void SegmentLabel::setOjects(const MyObject &object, const QString &sampleClass)
{
    if(maskImage != NULL)
    {
        delete maskImage;
        maskImage = NULL;
    }
    maskImage = new QImage(object.getSegmentImage());
    this->sampleClass = sampleClass;
    drawPixmap();
}

QList<MyObject> SegmentLabel::getSegment()
{
    QList<MyObject> allObject;
    allObject.clear();
    return allObject;
}

void SegmentLabel::drawPixmap()
{
    QPainter painter;
    tempPixmap = mp.copy();
    painter.begin(&tempPixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    if(maskImage != NULL)
    {
        painter.save();
        painter.setOpacity(0.5);
        painter.drawImage(QPoint(0, 0), *maskImage);
        painter.restore();
    }
    painter.end();
    this->update();
}

void SegmentLabel::initData()
{
    this->setMouseTracking(true);
    this->setCursor(Qt::CrossCursor);

    this->scale = 1.0f;
    this->maskImage = NULL;
    this->sampleClass = "All";
}

void SegmentLabel::initConnect()
{
}
