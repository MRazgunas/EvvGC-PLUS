#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QComboBox>
#include <QLabel>
#include <QMainWindow>
#include <QTimer>
#include <QtSerialPort/QSerialPort>
#include <QQuaternion>

#include <math.h>

/**
 * I2C bus error conditions
 */
#define I2C_NO_ERROR            0x00 /* No error.               */
#define I2C_BUS_ERROR           0x01 /* Bus Error.              */
#define I2C_ARBITRATION_LOST    0x02 /* Arbitration Lost.       */
#define I2C_ACK_FAILURE         0x04 /* Acknowledge Failure.    */
#define I2C_OVERRUN             0x08 /* Overrun/Underrun.       */
#define I2C_PEC_ERROR           0x10 /* PEC Error in reception. */
#define I2C_TIMEOUT             0x20 /* Hardware timeout.       */
#define I2C_SMB_ALERT           0x40 /* SMBus Alert.            */

#define RAD2DEG ( 180.0f / M_PI )

#define PWM_OUT_REV_FLAG        0x01
#define PWM_OUT_THI_FLAG        0x02

#define PWM_OUT_CMD_ID_MASK     0x0F
#define PWM_OUT_DT_ID_MASK      0xF0

typedef struct tagDataHdr
{
    quint8 cmd_id;
    quint8 size;
    const char *data;
    quint32 crc;
} __attribute__((packed)) DataHdr, *PDataHdr;

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
    quint8 flags;
    quint8 dt_cmd_id; /* High nibble contais dead-time ID, low nibble contains command ID. */
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

typedef struct tagI2CErrorStruct
{
    quint32 last_i2c_error;
    quint32 i2c_error_counter;
} __attribute__((packed)) I2CErrorStruct, *PI2CErrorStruct;

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
    void ProcessSerialCommands(const PDataHdr pHdr);
    void SendTelemetryData(const PDataHdr pHdr);
    quint32 GetCRC32Checksum(const PDataHdr pHdr);

private:
    Ui::MainWindow *ui;
    QComboBox *m_SerialDeviceList;
    QSerialPort m_serialPort;
    QTimer m_timer;
    char dataBuf[32];
    bool fConnected;
    quint8 bytesRequired;
    QQuaternion lastQ;
    DataHdr dataHdr;
    quint32 boardStatus;
    quint16 inputValues[5];
    float motorOffset[3];
};

#endif // MAINWINDOW_H
