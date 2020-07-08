#ifndef RECORDHISTORYDATA_H
#define RECORDHISTORYDATA_H

#include <QObject>
#include <QString>
#include <QList>

class RecordHistoryData : public QObject
{
    Q_OBJECT
public:
    RecordHistoryData(QObject *parent = 0);
    ~RecordHistoryData();

    QList<QString> getFileName(QList<QString> inputPaths);
    QList<QString> readHistoryData(const QString historyPathDir);
    void writeHistoryData(const QString savePathDir, const QList<QString> &recordDatas);

signals:

public slots:

private:

};

#endif // RECORDHISTORYDATA_H
