#ifndef IMAGECONVERTERWINDOW_H
#define IMAGECONVERTERWINDOW_H

#include <QWidget>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <QLineEdit>
#include <QList>
#include <QCloseEvent>
#include "videoTools/imageconverterthread.h"

class ImageConverterWindow : public QWidget
{
    Q_OBJECT

public:
    ImageConverterWindow(QWidget *parent = 0);
    ~ImageConverterWindow();

signals:
    void signalCloseImageConverterWindow(QString flag);

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
    QLabel *imagePostLabel;
    QComboBox *imagePostBox;
    QLabel *imageFormatLabel;
    QComboBox *imageFormatBox;

    QString pathDir;
    ImageConverterThread *imageConverterThread;

    void init();
    void initUI();
    void initConnect();
};

#endif // IMAGECONVERTERWINDOW_H
