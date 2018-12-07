#ifndef MARKCLASSWINDOW_H
#define MARKCLASSWINDOW_H

#include <QWidget>
#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QMap>

class MarkClassWindow : public QDialog
{
    Q_OBJECT

public:
    explicit MarkClassWindow(QDialog *parent = 0);
    ~MarkClassWindow();

    void setMarkClass(QMap<QString, QString> markClass);
    void setCalssName(QString name);
    void setClassClolor(QString color);
    QString getClassName();
    QString getClassColor();

signals:

public slots:

    void slotSelectColor();
    void slotOk();

private:
    QLabel *classNameLabel;
    QLineEdit *classNameText;
    QPushButton *classColorButton;
    QLabel *infoLabel;
    QPushButton *okButton;
    QPushButton *cancelButton;

    QMap<QString, QString> markClass;
    QString className;
    QString classColor;

private:

    void init();
    void initUI();
    void initConnect();
};

#endif // MARKCLASSWINDOW_H
