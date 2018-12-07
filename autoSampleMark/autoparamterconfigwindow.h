#ifndef AUTOPARAMTERCONFIGWINDOW_H
#define AUTOPARAMTERCONFIGWINDOW_H

#include <QWidget>
#include <QDialog>
#include <QSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include "utilityGUI/customWindow/modellabeltablewidget.h"
#include "autoparamterconfig.h"

class AutoParamterConfigWindow : public QDialog
{
    Q_OBJECT
public:
    explicit AutoParamterConfigWindow(QDialog *parent = 0);
    ~AutoParamterConfigWindow();

signals:

public slots:

    void slotOk();
    void slotSelectCaffeNet();
    void slotSelectCaffeModel();

protected:

    void closeEvent(QCloseEvent *event);

private:
    QLabel *caffeNetLabel;
    QLineEdit *caffeNetText;
    QPushButton *caffeNetButton;
    QLabel *caffeModelLabel;
    QLineEdit *caffeModelText;
    QPushButton *caffeModelButton;
    QLabel *inputDataWidthLabel;
    QSpinBox *inputDataWidthBox;
    QLabel *inputDataHeightLabel;
    QSpinBox *inputDataHeightBox;
    ModelLabelTableWidget *modelLabelTable;
    QPushButton *loadDefaultButton;
    QPushButton *saveButton;
    QPushButton *cancelButton;

    AutoParamterConfig paramterConfig;

private:

    void init();
    void initUI();
    void initConnect();

    void initTable();

    void loadDefaultValue();
};

#endif // AUTOPARAMTERCONFIGWINDOW_H
