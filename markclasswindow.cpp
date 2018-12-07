#pragma execution_character_set("utf-8")
#include "markclasswindow.h"
#include <QColorDialog>
#include <QMessageBox>
#include <QColor>
#include <QString>

MarkClassWindow::MarkClassWindow(QDialog *parent) : QDialog(parent)
{
    init();
    initUI();
    initConnect();
}

MarkClassWindow::~MarkClassWindow()
{

}

void MarkClassWindow::setMarkClass(QMap<QString, QString> markClass)
{
    this->markClass = markClass;
}

void MarkClassWindow::setCalssName(QString name)
{
    this->className = name;
    this->classNameText->setText(name);
}

void MarkClassWindow::setClassClolor(QString color)
{
    this->classColor = color;
    this->classColorButton->setStyleSheet(QString("background-color:%1").arg(color));
}

QString MarkClassWindow::getClassName()
{
    return this->className;
}

QString MarkClassWindow::getClassColor()
{
    return this->classColor;
}

void MarkClassWindow::slotSelectColor()
{
    QColor color;
    if(this->classColor != "")
    {
        color = QColorDialog::getColor(QColor(this->classColor));
    }
    else
    {
        color = QColorDialog::getColor(Qt::black);
    }
    if(color.isValid())
    {
        if(color.isValidColor("#000000"))
        {
            this->classColorButton->setStyleSheet(QString("background-color:%1").arg(color.name()));
            this->classColor = color.name();
        }
        else
        {
            QMessageBox::information(this, tr("颜色选择"), tr("不能选择黑色，黑色为错误标注类别的颜色"));
        }
    }
}

void MarkClassWindow::slotOk()
{
    this->className = this->classNameText->text().trimmed();
    if(this->className.isEmpty() && this->markClass.contains(this->className))
    {
        QMessageBox::information(this, tr("标注类别参数"), tr("%s标注类别存在").arg(this->className));
    }
    else
    {
        this->accept();
    }
}

void MarkClassWindow::init()
{
    this->className.clear();
    className = "";
    classColor = "";
}

void MarkClassWindow::initUI()
{
    classNameLabel = new QLabel(tr("标注类别名称："));
    classNameText = new QLineEdit();
    classColorButton = new QPushButton(tr("选择类别颜色"));

    QHBoxLayout *addClassLayout = new QHBoxLayout();
    addClassLayout->setSpacing(10);
    addClassLayout->addWidget(classNameLabel);
    addClassLayout->addWidget(classNameText);
    addClassLayout->addWidget(classColorButton);

    infoLabel = new QLabel(tr("注意：类别名称请使用英文命名"));

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
    mainLayout->addWidget(infoLabel);
    mainLayout->addLayout(bottomLayout);

    this->setLayout(mainLayout);
    this->setMaximumSize(300, 130);
    this->setMinimumSize(300, 130);
    this->setWindowTitle(tr("标注类别参数"));
}

void MarkClassWindow::initConnect()
{
    connect(classColorButton, &QPushButton::clicked, this, &MarkClassWindow::slotSelectColor);
    connect(okButton, &QPushButton::clicked, this, &MarkClassWindow::slotOk);
    connect(cancelButton, &QPushButton::clicked, this, &MarkClassWindow::reject);
}
