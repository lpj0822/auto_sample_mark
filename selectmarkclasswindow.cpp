#pragma execution_character_set("utf-8")
#include "selectmarkclasswindow.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QMessageBox>

#include "sampleMarkParam/manualparamterconfig.h"

SelectMarkClassWindow::SelectMarkClassWindow(QDialog *parent) : QDialog(parent)
{
    initData();
    initUI();
    initConncet();
}

QString SelectMarkClassWindow::getObjectClass()
{
    return classText->text();
}

bool SelectMarkClassWindow::getIsDifficult()
{
    return isDifficultBox->isChecked();
}

int SelectMarkClassWindow::getObjectFlag()
{
    return this->objectFlagBox->currentData().toInt();
}

void SelectMarkClassWindow::setObjectRect(const QString sampleClass)
{
    this->sampleCalss = sampleClass;
}

void SelectMarkClassWindow::slotSelectItem(QListWidgetItem *item)
{
    classText->setText(item->text());
}

void SelectMarkClassWindow::slotSelectOk(QListWidgetItem *item)
{
    classText->setText(item->text());
    if(this->sampleCalss.contains("All") || classText->text().contains(this->sampleCalss))
    {
        this->accept();
    }
    else
    {
        QMessageBox::information(this, tr("样本类别"), tr("您标注的样本类别有误！"));
    }
}

void SelectMarkClassWindow::slotOk()
{
    if(classText->text().trimmed() == "")
    {
        QMessageBox::information(this, tr("样本类别"), tr("请选择样本类别！"));
    }
    else
    {
        this->accept();

    }
}

void SelectMarkClassWindow::initData()
{
    classList.clear();
    QMap<QString, QString> markClassData = ManualParamterConfig::getMarkClass();
    for(QMap<QString, QString>::const_iterator classIterator = markClassData.constBegin();
        classIterator != markClassData.constEnd(); ++classIterator)
    {
        classList.append(classIterator.key());
    }
}

void SelectMarkClassWindow::initUI()
{
    classText = new QLineEdit();
    classText->setReadOnly(true);
    okButton = new QPushButton(tr("确定"));
    cancelButton = new QPushButton(tr("取消"));
    QHBoxLayout *topLayout = new QHBoxLayout();
    topLayout->setSpacing(20);
    topLayout->addWidget(classText);
    topLayout->addWidget(okButton);
    topLayout->addWidget(cancelButton);

    classListWidget = new QListWidget();
    //classListWidget->setStyleSheet("background-color:#FFFFFF");
    classListWidget->clear();
    for(int index = 0; index < classList.size(); index++)
    {
        QListWidgetItem *item = new QListWidgetItem(classList[index]);
        classListWidget->insertItem(index, item);
    }

    isDifficultBox = new QCheckBox(tr("是否是困难样本"));
    isDifficultBox->setCheckable(false);

    objectFlagLabel = new QLabel(tr("目标备注："));
    objectFlagBox = new QComboBox();
    initObjectFlagBox();
    QHBoxLayout *bottomLayout = new QHBoxLayout();
    bottomLayout->setSpacing(20);
    bottomLayout->addWidget(objectFlagLabel);
    bottomLayout->addWidget(objectFlagBox);

    flagGroundBox = new QGroupBox(tr("属性"));
    QVBoxLayout *flagLayout = new QVBoxLayout();
    flagLayout->setSpacing(10);
    flagLayout->addWidget(isDifficultBox);
    flagLayout->addLayout(bottomLayout);
    flagGroundBox->setLayout(flagLayout);

    QVBoxLayout *mainLayout = new QVBoxLayout();
    mainLayout->setSpacing(20);
    mainLayout->addLayout(topLayout);
    mainLayout->addWidget(classListWidget);
    mainLayout->addWidget(flagGroundBox);
    this->setLayout(mainLayout);
    this->setMaximumSize(200, 400);
    this->setMinimumSize(200, 400);
    this->setWindowTitle(tr("样本类别选择"));
}

void SelectMarkClassWindow::initConncet()
{
    connect(classListWidget, &QListWidget::itemClicked, this, &SelectMarkClassWindow::slotSelectItem);
    connect(classListWidget, &QListWidget::itemDoubleClicked, this, &SelectMarkClassWindow::slotSelectOk);
    connect(okButton, &QPushButton::clicked, this, &SelectMarkClassWindow::slotOk);
    connect(cancelButton, &QPushButton::clicked, this, &SelectMarkClassWindow::reject);
}

void SelectMarkClassWindow::initObjectFlagBox()
{
    for(int loop = 0; loop < 15; loop++)
    {
        objectFlagBox->addItem(QString("%1").arg(loop), loop);
    }
}
