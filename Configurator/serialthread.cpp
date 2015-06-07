#include "serialthread.h"

#include <QtSerialPort/QSerialPort>
#include <QDebug>

#define SERIAL_WRITE_TIMEOUT_MS         20
#define SERIAL_READ_TIMEOUT_MS          60
#define SERIAL_READ_TIMEOUT_EXTRA_MS    10

QT_USE_NAMESPACE

SerialThread::SerialThread(QObject *parent, int connectAttempts) :
    QThread(parent),
    m_quit(false),
    m_connectAttempts(connectAttempts)
{

}

SerialThread::~SerialThread()
{
    if (isRunning()) {
        disconnect();
    }
}

void SerialThread::connect(const QString &portName)
{
    m_mutex.lock();
    m_portName = portName;
    m_quit = false;
    if (!isRunning()) {
        start();
    } else {
        m_cond.wakeOne();
    }
    m_mutex.unlock();
}

void SerialThread::disconnect()
{
    m_mutex.lock();
    m_quit = true;
    m_cond.wakeOne();
    m_mutex.unlock();
    if (!wait(500)) {
        qDebug() << "Failed to terminate serial thread!";
    }
}

void SerialThread::run()
{
    QSerialPort serial;
    int cr;

    serial.setPortName(m_portName);
    if (!serial.setBaudRate(115200)) {
        qDebug() << "Serial set baud rate failed!";
        emit this->serialError(tr("Can't set baud rate for %1, error code %2. %3.")
            .arg(m_portName).arg(serial.error()).arg(serial.errorString()));
        return;
    }
    if (!serial.setDataBits(QSerialPort::Data8)) {
        qDebug() << "Serial set data bits failed!";
        emit this->serialError(tr("Can't set data bits for %1, error code %2. %3.")
            .arg(m_portName).arg(serial.error()).arg(serial.errorString()));
        return;
    }
    if (!serial.setParity(QSerialPort::NoParity)) {
        qDebug() << "Serial set parity failed!";
        emit this->serialError(tr("Can't set parity for %1, error code %2. %3.")
            .arg(m_portName).arg(serial.error()).arg(serial.errorString()));
        return;
    }
    if (!serial.setStopBits(QSerialPort::OneStop)) {
        qDebug() << "Serial set stop bits failed!";
        emit this->serialError(tr("Can't set stop bits for %1, error code %2. %3.")
            .arg(m_portName).arg(serial.error()).arg(serial.errorString()));
        return;
    }
    if (!serial.setFlowControl(QSerialPort::NoFlowControl)) {
        qDebug() << "Serial set flow control failed!";
        emit this->serialError(tr("Can't set flow control for %1, error code %2. %3.")
            .arg(m_portName).arg(serial.error()).arg(serial.errorString()));
        return;
    }
    for (cr = 0; cr < m_connectAttempts; cr++) {
        if (serial.open(QIODevice::ReadWrite)) {
            break;
        }
        sleep(1);
        qDebug() << "Serial connect retry...";
    }
    if (cr == m_connectAttempts) {
        qDebug() << "Connection failed!";
        emit this->serialError(tr("Can't open %1, error code %2. %3.")
            .arg(m_portName).arg(serial.error()).arg(serial.errorString()));
        return;
    }

    emit this->serialConnected();
    qDebug() << "Serial Thread is ready...";

    m_mutex.lock();
    /* Unlock resources and wait for the first job. */
    m_cond.wait(&m_mutex);
    m_mutex.unlock();

    while (!m_quit) {
        /* Protect shared resources while thread is working. */
        m_mutex.lock();

        if (m_txBuf.size() > 0) {
            qint64 bytesWritten = serial.write(m_txBuf);
            if (serial.waitForBytesWritten(SERIAL_WRITE_TIMEOUT_MS)) {
                m_txBuf.remove(0, bytesWritten);
            } else {
                qDebug() << "Write request timeout!";
                /* Unlock resources and exit. */
                m_mutex.unlock();
                emit serialTimeout(tr("Write request timeout!"));
                break;
            }
        }

        if (serial.waitForReadyRead(SERIAL_READ_TIMEOUT_MS)) {
            m_rxBuf += serial.readAll();
            while (serial.waitForReadyRead(SERIAL_READ_TIMEOUT_EXTRA_MS)) {
                m_rxBuf += serial.readAll();
            }
            if (m_rxBuf.size() >= TELEMETRY_MSG_SIZE_BYTES) {
                processInputData();
            }
        } else {
            qDebug() << "Read response timeout!";
            /* Unlock resources and exit. */
            m_mutex.unlock();
            emit serialTimeout(tr("Read response timeout!"));
            break;
        }

        //qDebug() << "Serial Thread is sleeping...";
        /* Unlock resources and wait for the next job. */
        m_cond.wait(&m_mutex);
        m_mutex.unlock();
    }

    qDebug() << "Serial Thread is terminating...";
    serial.close();
}

void SerialThread::write(const QByteArray &ba)
{
    m_mutex.lock();
    m_txBuf.append(ba);
    m_cond.wakeOne();
    m_mutex.unlock();
}

void SerialThread::processInputData()
{
    TelemetryMessage msg;
    const quint32 *checksum;
    quint8 msg_size;
    quint8 crc_size;

    if ((quint8)m_rxBuf.at(TELEMETRY_MSG_SOF_ID) == TELEMETRY_MSG_SOF) {
        msg_size = (quint8)m_rxBuf.at(TELEMETRY_MSG_SIZE_ID);
        if ((msg_size >= TELEMETRY_MSG_SIZE_BYTES) && (msg_size <= TELEMETRY_MSG_SIZE_BYTES_MAX)) {
            if (m_rxBuf.size() >= msg_size) {
                memset((void *)&msg, 0, sizeof(msg));
                /* Compute size of data in bytes for CRC calculation. */
                msg_size -= TELEMETRY_CRC_SIZE_BYTES;
                /* Copy data to zero padded CRC buffer. */
                memcpy((void *)&msg, (void *)m_rxBuf.constData(), msg_size);
                /* Get checksum member. */
                checksum = (quint32 *)(m_rxBuf.constData() + msg_size);
                /* Align CRC data size to 32-bit boundaries. */
                crc_size = msg_size / sizeof(quint32);
                if (msg_size % sizeof(quint32)) {
                    crc_size++;
                }
                if (*checksum == crc32((quint32 *)&msg, crc_size)) {
                    msg.crc = *checksum;
                    emit this->serialDataReady(msg);
                } else {
                    qDebug() << "CRC error detected!";
                }
                m_rxBuf.remove(0, m_rxBuf.at(TELEMETRY_MSG_SIZE_ID));
                if (m_rxBuf.size() >= TELEMETRY_MSG_SIZE_BYTES) {
                    processInputData();
                }
            }
        } else {
            qDebug() << "Message size error detected!";
            /* Remove message with bad size. */
            m_rxBuf.remove(0, TELEMETRY_MSG_SIZE_BYTES);
            if (m_rxBuf.size() >= TELEMETRY_MSG_SIZE_BYTES) {
                processInputData();
            }
        }
    } else {
        qDebug() << "Message SOF error detected!";
        /* Try to find the next SOF. */
        do {
            m_rxBuf.remove(0, 1);
        } while (m_rxBuf.size() && ((quint8)m_rxBuf.at(TELEMETRY_MSG_SOF_ID) != TELEMETRY_MSG_SOF));
        if (m_rxBuf.size() >= TELEMETRY_MSG_SIZE_BYTES) {
            processInputData();
        }
    }
}
