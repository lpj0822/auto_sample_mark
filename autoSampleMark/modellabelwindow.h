#ifndef MODELLABELWINDOW_H
#define MODELLABELWINDOW_H

#include <QWidget>
#include <QDialog>
#include <QLabel>
#include <QSpinBox>
#include <QComboBox>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QMap>

class ModelLabelWindow: public QDialog
{
    Q_OBJECT

public:
    explicit ModelLabelWindow(QDialog *parent = 0);
    ~ModelLabelWindow();

    void setModelLabel(QMap<int, QString> modelLabel);
    void setLabelId(int id);
    void setLabelName(QString name);
    int getLabelId();
    QString getLabelName();

signals:

public slots:

    void slotOk();

private:
    QLabel *labelIdLabel;
    QSpinBox *labelIdBox;
    QLabel *labelNameLabel;
    QComboBox *labelNameBox;
    QPushButton *okButton;
    QPushButton *cancelButton;

    QMap<int, QString> modelLabel;

private:

    void init();
    void initUI();
    void initConnect();
};


#endif // MODELLABELWINDOW_H
