#include "wexpand.h"
#include <QVBoxLayout>
#include <QDebug>
#include "myhelper.h"

WExpand::WExpand(QWidget *parent) : QFrame(parent),
    expanded(true)
{
    QtAwesome* awesome = MyHelper::getAwesome();
    btn = new QToolButton(this);
    btn->setObjectName("WExpand_btn");
    btn->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);
    btn->setMaximumWidth(4);
    btn->setText(QChar(fa::angleleft));
    btn->setFont(awesome->font(16));

    QVBoxLayout *mainLayout = new QVBoxLayout();
    mainLayout->setMargin(0);
    mainLayout->addStretch(0);
    mainLayout->addWidget(btn);
    mainLayout->addStretch(0);
    this->setLayout(mainLayout);
    this->setMinimumWidth(1);
}

void WExpand::setAngle(bool angle)
{
    if(!angle)
    {
        btn->setText(QChar(fa::angleleft));
        connect(btn,&QToolButton::clicked,[&](){
            if(expanded){
                btn->setText(QChar(fa::angleright));
            }
            else {
                btn->setText(QChar(fa::angleleft));
            }
            expanded = !expanded;
            emit signalStatusChangeed(expanded);
        });
    }
    else
    {
        btn->setText(QChar(fa::angleright));
        connect(btn,&QToolButton::clicked,[&](){
            if(expanded){
                btn->setText(QChar(fa::angleleft));
            }
            else {
                btn->setText(QChar(fa::angleright));
            }
            expanded = !expanded;
            emit signalStatusChangeed(expanded);
        });
    }
}
