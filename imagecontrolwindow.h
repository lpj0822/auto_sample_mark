#ifndef IMAGECONTROLWINDOW_H
#define IMAGECONTROLWINDOW_H

#include "controlwindow.h"
#include "utilityGUI/customWindow/myscrollarea.h"
#include "drawWidget/editablelabel.h"

class ImageControlWindow : public ControlWindow
{
    Q_OBJECT

public:
    ImageControlWindow(QWidget *parent = 0);
    virtual ~ImageControlWindow() override;

    void setMarkDataList(const QString markDataDir, const QList<QString> markDataList, const MarkDataType dataType) override;
    void saveMarkDataList() override;
    void setDrawShape(int shapeId) override;

public slots:

    void slotIsMark();
    void slotImageItem(QListWidgetItem *item);
    void slotChangeClass(QString classText);
    virtual void slotScrollArea(int keyValue);

protected:
    void closeEvent(QCloseEvent *event) override;
    virtual void keyPressEvent(QKeyEvent *e) override;

protected:

    virtual void showPrevious();
    virtual void showNext();

    void updateDrawLabel(bool isValue);
    void updateImage();

    virtual void loadMarkData(const QString dataPath);
    virtual void saveMarkDataResult();
    virtual void loadMarkImage();
    virtual void saveMarkImageResult();

    void initDrawWidget();
    void initConnect();
    void initImageList();

protected:
    EditableLabel *drawLable;
    QScrollArea *drawLableScrollArea;

private:
    //imageData
    void loadImageData(const QString imagePath, const QString saveAnnotationsDir);
    void saveImageDataResult(const QString &saveAnnotationsDir, const QString &imagePath, const QList<MyObject> &objects);
    void saveImageSegmentResult(const QString &saveAnnotationsDir, const QString &imagePath, const QList<MyObject> &objects);

    void initData();
    void initImageData();

private:
    //imageData
    QString currentImagePath;

};

#endif // IMAGECONTROLWINDOW_H
