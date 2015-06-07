#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QMainWindow>
#include <QTimer>
#include <QQuaternion>

#include <math.h>

#include "ffft/FFTRealFixLen.h"

#include "serialthread.h"
#include "telemetry.h"
#include "crc32.h"

#define SERIAL_CONNECT_ATTEMPTS  (20)

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

#define SENSOR1_AXIS_DIR_POS    0x08
#define SENSOR1_AXIS_ID_MASK    0x07
#define SENSOR2_AXIS_DIR_POS    0x80
#define SENSOR2_AXIS_ID_MASK    0x70

/* Each pass of the FFT processes 2^X samples, where X is the
 * number below.
 */
static const int FFTLengthPowerOfTwo = 8;
static const int FFTLengthNumSamples = 256; /* 2^8 = 256 */

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
    quint8 dt_cmd_id; /* High nibble contains dead-time ID, low nibble contains command ID. */
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
    void SerialConnect();
    void SerialConnected();
    void ProcessSerialMessages(const TelemetryMessage &msg);
    void SerialError(const QString &s);
    void HandleReadSettings();
    void HandleApplySettings(bool warnDeadTimeChange = true);
    void HandleSaveSettings();
    void ProcessTimeout();
    void HandleDataXClicked();
    void HandleDataYClicked();
    void HandleDataZClicked();
    void HandleAccCalibrate();
    void HandleGyroCalibrate();
    void HandleSpectrumSourceChanged(int id);

private:
    void FillPortsInfo();
    void UpdatePlotData(const float xyz[3]);
    void UpdateSpectrum();
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
    bool GetCFSettings();
    bool SetCFSettings();
    quint32 GetCRC32Checksum(const TelemetryMessage &msg);
    void SerialDataWrite(const TelemetryMessage &msg);
    void SerialDataWrite(quint8 msgId, void *buf, int bufLen);

private:
    Ui::MainWindow *ui;
    QComboBox *m_serialDeviceList;
    SerialThread m_serialThread;
    QTimer m_timer;
    bool m_fConnected;
    TelemetryMessage m_msg;
    quint32 m_boardStatus;
    quint16 m_inputValues[5];
    float m_motorOffset[3];
    QCheckBox *m_i2cStatus;
    bool m_deadtimeChanged;
    quint32 m_dataInIdx;
    float m_dataIn[FFTLengthNumSamples];
    float m_dataOut[FFTLengthNumSamples];
    QVector<double> m_amplitude;
    QVector<double> m_frequency;
    ffft::FFTRealFixLen <FFTLengthPowerOfTwo> m_fft;
};

#endif // MAINWINDOW_H
