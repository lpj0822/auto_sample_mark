#pragma execution_character_set("utf-8")
#include "modellabelwindow.h"
#include <QMessageBox>
#include <QString>
#include <QStringList>
#include "manualparamterconfig.h"

ModelLabelWindow::ModelLabelWindow(QDialog *parent) : QDialog(parent)
{
    init();
    initUI();
    initConnect();
}

ModelLabelWindow::~ModelLabelWindow()
{

}

void ModelLabelWindow::setModelLabel(QMap<int, QString> modelLabel)
{
    this->modelLabel = modelLabel;
}

void ModelLabelWindow::setLabelId(int id)
{
    this->labelIdBox->setValue(id);
}

void ModelLabelWindow::setLabelName(QString name)
{
    this->labelNameBox->setCurrentText(name);
}

int ModelLabelWindow::getLabelId()
{
    return this->labelIdBox->value();
}

QString ModelLabelWindow::getLabelName()
{
    return this->labelNameBox->currentText();
}

void ModelLabelWindow::slotOk()
{
    int id = this->labelIdBox->value();
    QString name = this->labelNameBox->currentText();
    if(this->modelLabel.keys().contains(id) || this->modelLabel.values().contains(name))
    {
        QMessageBox::information(this, tr("模型标签参数"), tr("模型标签ID:%d存在,或者标签名称:%s存在").arg(id).arg(name));
    }
    else
    {
        this->accept();
    }
}

void ModelLabelWindow::init()
{
    this->modelLabel.clear();
}

void ModelLabelWindow::initUI()
{
    labelIdLabel = new QLabel(tr("标签ID："));
    labelIdBox = new QSpinBox();
    labelIdBox->setSingleStep(1);
    labelIdBox->setMinimum(0);
    labelIdBox->setValue(0);
    labelNameLabel = new QLabel(tr("标签名称："));
    labelNameBox = new QComboBox();
    QList<QString> names = ManualParamterConfig::getMarkClass().values();
    labelNameBox->addItems(QStringList(names));
    labelNameBox->setCurrentIndex(0);

    QHBoxLayout *addClassLayout = new QHBoxLayout();
    addClassLayout->setSpacing(10);
    addClassLayout->addWidget(labelIdLabel);
    addClassLayout->addWidget(labelIdBox);
    addClassLayout->addWidget(labelNameLabel);
    addClassLayout->addWidget(labelNameBox);

    okButton = new QPushButton(tr("确定"));
    cancelButton = new QPushButton(tr("取消"));

    QHBoxLayout *bottomLayout = new QHBoxLayout();
    bottomLayout->setSpacing(20);
    bottomLayout->setAlignment(Qt::AlignRight);
    bottomLayout->addWidget(okButton);
    bottomLayout->addWidget(cancelButton);

    QVBoxLayout *mainLayout = new QVBoxLayout();
    mainLayout->setSpacing(10);
    mainLayout->addLayout(addClassLayout);
    mainLayout->addLayout(bottomLayout);

    this->setLayout(mainLayout);
    this->setMaximumSize(300, 100);
    this->setMinimumSize(300, 100);
    this->setWindowTitle(tr("标注类别参数"));
}

void ModelLabelWindow::initConnect()
{
    connect(okButton, &QPushButton::clicked, this, &ModelLabelWindow::slotOk);
    connect(cancelButton, &QPushButton::clicked, this, &ModelLabelWindow::reject);
}

