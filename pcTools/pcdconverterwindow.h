#ifndef PCDCONVERTERWINDOW_H
#define PCDCONVERTERWINDOW_H

#include <QWidget>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <QSpinBox>
#include <QLineEdit>
#include <QList>
#include <QCloseEvent>
#include "pcdconverterthread.h"

class PCDConverterWindow : public QWidget
{
    Q_OBJECT

public:
    PCDConverterWindow(QWidget *parent = 0);
    ~PCDConverterWindow();

signals:
    void signalClosePCDConverterWindow(QString flag);

public slots:
    void slotOpen();
    void slotStart();
    void slotStop();
    void slotFinish(QString name);

protected:
    void closeEvent(QCloseEvent *event);

private:

    QVBoxLayout *mainLayout;
    QPushButton *openButton;
    QPushButton *startButton;
    QPushButton *stopButton;
    QLineEdit *pathText;
    QLabel *fieldsNumberLabel;
    QSpinBox *fieldsNumberBox;
    QLabel *formatLabel;
    QComboBox *formatBox;

    QString pathDir;
    PCDConverterThread *pcdConverterThread;

    void init();
    void initUI();
    void initConnect();
};

#endif // PCDCONVERTERWINDOW_H
