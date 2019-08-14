#ifndef IMAGESEGMENTCONTROLWINDOW_H
#define IMAGESEGMENTCONTROLWINDOW_H

#include "controlwindow.h"
#include "utilityGUI/customWindow/myscrollarea.h"
#include "drawWidget/segmentlabel.h"

class ImageSegmentControlWindow : public ControlWindow
{
    Q_OBJECT

public:
    ImageSegmentControlWindow(QWidget *parent = 0);
    ~ImageSegmentControlWindow() override;

    void setMarkDataList(const QString markDataDir, const QList<QString> markDataList, const MarkDataType dataType) override;
    void saveMarkDataList() override;
    void setDrawShape(int shapeId) override;

public slots:

    void slotImageItem(QListWidgetItem *item);
    void slotChangeClass(QString classText);
    void slotScrollArea(int keyValue);

protected:
    void closeEvent(QCloseEvent *event) override;
    void keyPressEvent(QKeyEvent *e) override;

protected:

    void showPrevious();
    void showNext();

    void updateImage();

    void loadMarkData(const QString dataPath);
    void loadMarkImage();

    void initDrawWidget();
    void initConnect();
    void initImageList();

protected:
    SegmentLabel *drawLable;
    QScrollArea *drawLableScrollArea;

private:
    //imageData
    void loadImageData(const QString imagePath, const QString saveAnnotationsDir);

    void initData();
    void initImageData();

private:
    //imageData
    QString currentImagePath;

};

#endif // IMAGESEGMENTCONTROLWINDOW_H
