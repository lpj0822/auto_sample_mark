#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "mainwindow.h"
#include <QApplication>
#include <QTextCodec>
#include <QFile>
#include <QTranslator>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    //QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));
    app.setApplicationVersion("1.0");
    app.setApplicationName("样本标注系统");

    QTranslator translate;
    translate.load(":/qm/zh.qm");
    app.installTranslator(&translate);

    //load qss file
    QFile file(":/style/style/style.qss");
    file.open(QFile::ReadOnly);
    QString qss = QLatin1String(file.readAll());
    qApp->setStyleSheet(qss);

    MainWindow w;
    w.show();

    return app.exec();
}
