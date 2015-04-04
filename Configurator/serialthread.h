#ifndef SERIALTHREAD_H
#define SERIALTHREAD_H

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QtSerialPort/QSerialPort>

#include "telemetry.h"
#include "crc32.h"

class SerialThread : public QThread
{
    Q_OBJECT

public:
    SerialThread(QObject *parent = 0, int connectAttempts = 1);
    ~SerialThread();

    void connect(const QString &portName);
    void disconnect();
    void write(const QByteArray &ba);

protected:
    void run() Q_DECL_OVERRIDE;

signals:
    void serialConnected();
    void serialError(const QString &s);
    void serialTimeout(const QString &s);
    void serialDataReady(const TelemetryMessage &msg);

private:
    void processInputData();

private:
    QString m_portName;
    QMutex m_mutex;
    QWaitCondition m_cond;
    QByteArray m_txBuf;
    QByteArray m_rxBuf;
    bool m_quit;
    int m_connectAttempts;
};

#endif // SERIALTHREAD_H
