#ifndef CONTROLWINDOW_H
#define CONTROLWINDOW_H

#include <QWidget>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QTextBrowser>
#include <QCheckBox>
#include <QProgressBar>
#include <QListWidget>
#include <QGroupBox>
#include <QAction>
#include <QScrollArea>
#include <QToolButton>
#include <QMap>

#include "helpers/recordhistorydata.h"
#include "saveMarkData/xmlprocess.h"
#include "saveMarkData/jsonprocessvideo.h"
#include "saveMarkData/segmentimagesave.h"
#include "utilityGUI/customWindow/wexpand.h"
#include "utilityGUI/customWindow/customanimation.h"
#include "utilityGUI/customWindow/mystackedwidget.h"

typedef enum MarkDataType{
    UNKNOWN = 0,
    IMAGE = 1,
    VIDEO = 2,
    SEGMENT = 3,
    PCD = 4,
    MAX_CONUT
}MarkDataType;


class ControlWindow : public QWidget
{
    Q_OBJECT

public:
    ControlWindow(QWidget *parent = 0);
    virtual ~ControlWindow();

    virtual void setMarkDataList(const QString markDataDir, const QList<QString> markDataList, const MarkDataType dataType);
    virtual void saveMarkDataList();
    virtual void setDrawShape(int shapeId);

public slots:
    void slotManualMarkParamterChanged();
    void slotShowFull();

protected:
    void resizeEvent(QResizeEvent *e);
    void contextMenuEvent (QContextMenuEvent * event);

protected:

    void updateIsMarkButton(bool isValue);
    void updateListBox();
    void updateMarkProcessLable();
    virtual void updateLabelText(int markCount);

    void init();
    void initUI();
    void initMarkData(const QString dirPath, const MarkDataType dataType);
    void initMarkClassBox();

    void initExpandLeft();
    void initExpandRight();
    void updateExpandLeft();
    void updateExpandRight();

    void readClassConfig(const QString &markDataDir);
    void readMarkHistory();
    void writeMarkHistory();

protected:

    QGroupBox *centerTopBox;

    QWidget *centerWidget;
    QLabel *showClass;
    QComboBox *classBox;
    QPushButton *showFullButton;
    QPushButton *isMarkButton;
    QLabel *markProcessLabel;

    MyStackedWidget *drawMarkDataWidget;
    QLabel *baseLabel;
    QScrollArea *lableScrollArea;
    QListWidget *markDataListWidget;

    WExpand *expand1;
    CustomAnimation *customAnimation1;
    bool leftTabXxpanded1;
    int leftTabminmumwidth1;
    WExpand *expand2;
    CustomAnimation *customAnimation2;
    bool leftTabXxpanded2;
    int leftTabminmumwidth2;

protected:

    QList<QString> processMarkDataList;
    QList<int> processDataFlagList;

    QString markDataDir;
    bool isMark;
    MarkDataType markDataType;

    QImage currentImage;
    int currentIndex;

    JSONProcessVideo jsonProcess;
    XMLProcess xmlProcess;
    SegmentImageSave segmentImageProcess;

    RecordHistoryData historyProcess;
};

#endif // CONTROLWINDOW_H
