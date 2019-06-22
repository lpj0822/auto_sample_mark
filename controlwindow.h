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
#include "utilityGUI/customWindow/wexpand.h"
#include "utilityGUI/customWindow/customanimation.h"
#include "utilityGUI/customWindow/myscrollarea.h"
#include "utilityGUI/customWindow/mystackedwidget.h"
#include "editablelabel.h"

typedef enum MarkDataType{
    UNKNOWN = 0,
    IMAGE = 1,
    VIDEO = 2
}MarkDataType;


class ControlWindow : public QWidget
{
    Q_OBJECT

public:
    ControlWindow(QWidget *parent = 0);
    ~ControlWindow();

    void setMarkDataList(const QString markDataDir, const QList<QString> markDataList, const MarkDataType dataType);
    void setDrawShape(int shapeId);

public slots:
    void slotManualMarkParamterChanged();

protected:
    void resizeEvent(QResizeEvent *e);
    void contextMenuEvent (QContextMenuEvent * event);

protected:

    void updateIsMarkButton(bool isValue);
    void updateListBox();

    void init();
    void initUI();
    void initMarkData(const QString dirPath, const MarkDataType dataType);
    void initMarkClassBox();
    void initDrawWidget();

    void initExpandLeft();
    void initExpandRight();
    void updateExpandLeft();
    void updateExpandRight();

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
    EditableLabel *drawLable;
    QScrollArea *drawLableScrollArea;
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
    RecordHistoryData historyProcess;
};

#endif // CONTROLWINDOW_H
