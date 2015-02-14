#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QComboBox>
#include <QLabel>
#include <QMainWindow>
#include <QTimer>
#include <QtSerialPort/QSerialPort>
#include <QQuaternion>

#include <math.h>

#define RAD2DEG ( 180.0f / M_PI )

typedef struct tagDataHdr
{
    quint8 cmd_id;
    quint8 size;
    const char *data;
    quint32 crc;
} __attribute__((packed)) DataHdr, *PDataHdr;

typedef struct tagListItem
{
    quint8 item_id;
    const char item_name[12];
} __attribute__((packed)) ListItem, *PListItem;

typedef struct tagPIDSettings
{
    quint8 P;
    quint8 I;
    quint8 D;
} __attribute__((packed)) PIDSettings, *PPIDSettings;

typedef struct tagOutputSettings
{
    quint8 power;
    quint8 num_poles;
    quint8 reverse;
    quint8 cmd_id;
    quint8 dt_id;
} __attribute__((packed)) OutputSettings, *POutputSettings;

typedef struct tagInputSettings
{
    quint16 min_val;
    quint16 mid_val;
    quint16 max_val;
    quint8 channel_id;
} __attribute__((packed)) InputSettings, *PInputSettings;

typedef struct tagInputModeSettings
{
    qint16 min_angle;
    qint16 max_angle;
    qint16 offset;
    quint8 speed;
    quint8 mode_id;
} __attribute__((packed)) InputModeSettings, *PInputModeSettings;

typedef struct tagSensorSettings
{
    quint8 axis_id;
    qint8 axis_dir;
} __attribute__((packed)) SensorSettings, *PSensorSettings;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void HandleSerialConnection();
    void HandleReadSettings();
    void HandleApplySettings();
    void HandleSaveSettings();
    void ProcessTimeout();
    void ReadSerialData();
    void HandleSerialError(QSerialPort::SerialPortError error);
    void HandleDataXClicked();
    void HandleDataYClicked();
    void HandleDataZClicked();
    void HandleAccCalibrate();
    void HandleGyroCalibrate();

private:
    void FillPortsInfo();
    void ProcessSerialCommands(const PDataHdr pHdr);
    bool GetStabilizationSettings();
    bool SetStabilizationSettings();
    bool GetOutputSettings();
    bool SetOutputSettings();
    bool GetInputSettings();
    bool SetInputSettings();
    bool GetInputModeSettings();
    bool SetInputModeSettings();
    bool GetSensorSettings();
    bool SetSensorSettings();
    void SendTelemetryData(const PDataHdr pHdr);
    quint32 GetCRC32Checksum(const PDataHdr pHdr);

private:
    Ui::MainWindow *ui;
    QComboBox *cbDevList;
    QSerialPort *serial;
    QLabel *lbI2CErrors;
    QTimer timer;
    char dataBuf[32];
    bool fConnected;
    quint8 bytesRequired;
    QQuaternion lastQ;
    DataHdr dataHdr;
    quint32 boardStatus;
    quint16 inputValues[5];
};

#endif // MAINWINDOW_H
