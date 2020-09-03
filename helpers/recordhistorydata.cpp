#pragma execution_character_set("utf-8")
#include "recordhistorydata.h"
#include <QMessageBox>
#include <QDir>
#include <QFileInfo>
#include <QFile>
#include <QTextStream>

RecordHistoryData::RecordHistoryData(QObject *parent) : QObject(parent)
{

}

RecordHistoryData::~RecordHistoryData()
{

}

QList<QString> RecordHistoryData::getFileName(QList<QString> inputPaths)
{
    QList<QString> result;
    for(int index = 0; index < inputPaths.size(); index++)
    {
        QFileInfo fileInfo(inputPaths[index]);
        result.append(fileInfo.fileName());
    }
    return result;
}

QList<QString> RecordHistoryData::readHistoryData(const QString historyPathDir)
{
    QList<QString> result;
    QString historyDataFile = historyPathDir + "/" + ".history";
    QFileInfo fileInfo(historyDataFile);
    result.clear();
    if(fileInfo.exists())
    {
        QFile file(historyDataFile);
        if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
          QMessageBox::warning(NULL, tr("Warnning"), tr("读文件时，不能打开文件：%1").arg(historyDataFile), QMessageBox::Yes);
        }
        else
        {
            QTextStream in(&file);
            while (!in.atEnd())
            {
                QString data = in.readLine().trimmed();
                if(!data.isEmpty() && data != "")
                {
                    result.append(data);
                }
            }
        }
    }
    return result;
}

void RecordHistoryData::writeHistoryData(const QString savePathDir, const QList<QString> &recordDatas)
{
    int count = recordDatas.size();
    if(count > 0)
    {
        QString historyDataFile = savePathDir + "/" + ".history";
        QFile file(historyDataFile);
        if(!file.open(QIODevice::WriteOnly))
        {
            QMessageBox::warning(NULL, tr("Warnning"), tr("写文件时，不能打开文件：%1").arg(historyDataFile), QMessageBox::Yes);
        }
        else
        {
            QTextStream out(&file);
            for(int index = 0; index < count; index++)
            {
                out << recordDatas[index].trimmed() << endl;
            }
            file.close();
        }
    }
}
