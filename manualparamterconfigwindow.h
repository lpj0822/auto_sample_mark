#ifndef MANUALPARAMTERCONFIGWINDOW_H
#define MANUALPARAMTERCONFIGWINDOW_H

#include <QWidget>
#include <QDialog>
#include <QSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include "utilityGUI/customWindow/markclasstablewidget.h"
#include "manualparamterconfig.h"

class ManualParamterConfigWindow : public QDialog
{
    Q_OBJECT
public:
    explicit ManualParamterConfigWindow(QDialog *parent = 0);
    ~ManualParamterConfigWindow();

signals:

public slots:

    void slotOk();

protected:

    void closeEvent(QCloseEvent *event);

private:
    QLabel *minWidthLabel;
    QSpinBox *minWidthBox;
    QLabel *minHeightLabel;
    QSpinBox *minHeightBox;
    MarkClassTableWidget *markClassTable;
    QPushButton *loadDefaultButton;
    QPushButton *saveButton;
    QPushButton *cancelButton;

    ManualParamterConfig paramterConfig;

private:

    void init();
    void initUI();
    void initConnect();

    void initTable();

    void loadDefaultValue();
};

#endif // MANUALPARAMTERCONFIGWINDOW_H
