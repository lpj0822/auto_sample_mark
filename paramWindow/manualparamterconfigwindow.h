#ifndef MANUALPARAMTERCONFIGWINDOW_H
#define MANUALPARAMTERCONFIGWINDOW_H

#include <QWidget>
#include <QDialog>
#include <QSpinBox>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QStackedWidget>
#include "utilityGUI/customWindow/markclasstablewidget.h"
#include "utilityGUI/customWindow/markclasstreewidget.h"
#include "sampleMarkParam/manualparamterconfig.h"

class ManualParamterConfigWindow : public QDialog
{
    Q_OBJECT
public:
    explicit ManualParamterConfigWindow(QDialog *parent = 0);
    ~ManualParamterConfigWindow();

signals:

public slots:

    void slotOk();
    void slotClassModelSelect(QString text);

protected:
    void closeEvent(QCloseEvent *event);

private:
    QLabel *classModelSelectLabel;
    QComboBox *classModelSelectBox;
    QLabel *minWidthLabel;
    QSpinBox *minWidthBox;
    QLabel *minHeightLabel;
    QSpinBox *minHeightBox;

    QStackedWidget *classModelWidget;
    MarkClassTableWidget *markClassTable;
    MarkClassTreeWidget *markClassTree;

    QPushButton *loadDefaultButton;
    QPushButton *saveButton;
    QPushButton *cancelButton;

    ManualParamterConfig paramterConfig;

private:

    void init();
    void initUI();
    void initConnect();

    void initModelSelect();
    void initTable();
    void initTree();

    void loadDefaultValue();
};

#endif // MANUALPARAMTERCONFIGWINDOW_H
