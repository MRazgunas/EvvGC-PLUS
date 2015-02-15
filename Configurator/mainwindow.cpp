#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QComboBox>
#include <QtSerialPort/QSerialPortInfo>
#include <QDebug>

static ListItem OutputCommands[4] = {
/*  ID,  Name */
    {0, "Pitch"   },
    {1, "Roll"    },
    {2, "Yaw"     },
    {3, "Disabled"},
};

static ListItem OutputDeadTime[5] = {
/*  ID,  Name */
    {0, "1000ns" },
    {1, "2000ns" },
    {2, "3000ns"},
    {3, "4000ns"},
    {4, "5000ns"},
};

static ListItem InputChannel[6] = {
/*  ID, Name */
    {0, "AUX1 analog"},
    {1, "AUX2 analog"},
    {2, "AUX3 PWM"},
    {3, "AUX4 PWM"},
    {4, "AUX5 PWM"},
    {5, "Disabled"},
};

static ListItem InputMode[3] = {
/*  ID, Name */
    {0, "Angle"},
    {1, "Speed"},
    {2, "Follow"},
};

static ListItem SensorAxis[6] = {
/*  ID, Name */
    {0, "+X"},
    {1, "+Y"},
    {2, "+Z"},
    {3, "-X"},
    {4, "-Y"},
    {5, "-Z"},
};

static ListItem PlotData[1] = {
/*  ID, Name */
    {0, "Attitude"}
};

static PIDSettings PID[3] = {
/*   P, I, D */
    {0, 0, 0},  // Pitch
    {0, 0, 0},  // Roll
    {0, 0, 0}   // Yaw
};

static OutputSettings outSettings[3] = {
    {0,     /* Power, % */
     14,    /* Number of poles */
     0,     /* Reverse */
     3,     /* Command ID */
     4},    /* Dead-time ID */
    {0,     /* Power, % */
     14,    /* Number of poles */
     0,     /* Reverse */
     3,     /* Command ID */
     4},    /* Dead-time ID */
    {0,     /* Power, % */
     14,    /* Number of poles */
     0,     /* Reverse */
     3,     /* Command ID */
     4}     /* Dead-time ID */
};

static InputSettings inSettings[3] = {
    {1000,  /* Min */
     1500,  /* Mid */
     2000,  /* Max */
     5},    /* Channel # */
    {1000,  /* Min */
     1500,  /* Mid */
     2000,  /* Max */
     5},    /* Channel # */
    {1000,  /* Min */
     1500,  /* Mid */
     2000,  /* Max */
     5}     /* Channel # */
};

static InputModeSettings modeSettings[3] = {
    {-60,   /* Min angle */
     60,    /* Max angle */
     0,     /* Offset */
     20,    /* Speed */
     0},    /* Mode ID */
    {-60,   /* Min angle */
     60,    /* Max angle */
     0,     /* Offset */
     20,    /* Speed */
     0},    /* Mode ID */
    {-90,   /* Min angle */
     90,    /* Max angle */
     0,     /* Offset */
     20,    /* Speed */
     0}     /* Mode ID */
};

static SensorSettings sensorSettings[3] = {
/*  ID, DIR */
    {0,  1}, /* Pitch (X) */
    {1, -1}, /* Roll (Y)  */
    {2, -1}  /* Yaw (Z)   */
};

static quint8 sensorAxes[3] = {
    3, /* Right axis -X; */
    4, /* Front axis -Y; */
    2  /* Top axis   +Z; */
};

/* Axes mapping matrix. */
static quint8 amMtx[6][6] = {
//T:+X,+Y,+Z,-X,-Y,-Z  //R:
    {6, 5, 1, 6, 2, 4},//+X;
    {2, 6, 3, 5, 6, 0},//+Y;
    {4, 0, 6, 1, 3, 6},//+Z;
    {6, 2, 4, 6, 5, 1},//-X;
    {5, 6, 0, 2, 6, 3},//-Y;
    {1, 3, 6, 4, 0, 6},//-Z;
};

/**
 * @brief Quaternion2RPY
 * @param q
 * @param rpy
 */
static inline void Quaternion2RPY(const float q[4], float rpy[3])
{
    float R13, R11, R12, R23, R33;
    float q2s = q[2]*q[2];

    R11 = 1.0f - 2.0f * (q2s + q[3]*q[3]);
    R12 = 2.0f * (q[0]*q[3] + q[1]*q[2]);
    R13 = 2.0f * (q[0]*q[2] - q[1]*q[3]);
    R23 = 2.0f * (q[0]*q[1] + q[2]*q[3]);
    R33 = 1.0f - 2.0f * (q2s + q[1]*q[1]);

    rpy[1] = asinf (R13);   // roll always between -pi/2 to pi/2
    rpy[2] = atan2f(R12, R11);
    rpy[0] = atan2f(R23, R33);

    //TODO: consider the cases where |R13| ~= 1, |roll| ~= pi/2
}

/**
 * @brief crc32 - calculate CRC32 checksum using STM32 algorythm.
 * @param pBuf - data buffer.
 * @param length - length of the data buffer.
 * @return CRC32 checksum.
 */
quint32 crc32(const quint32 pBuf[], size_t length);

/**
 * @brief MainWindow::MainWindow
 * @param parent
 */
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    fConnected(false),
    bytesRequired(0),
    boardStatus(0)
{
    ui->setupUi(this);
    cbDevList = new QComboBox(ui->mainToolBar);
    cbDevList->setMinimumWidth(250);
    cbDevList->setEnabled(false);
    ui->mainToolBar->insertWidget(ui->actionHandleConnection, cbDevList);
    ui->mainToolBar->insertSeparator(ui->actionHandleConnection);
    lbI2CErrors = new QLabel(ui->statusBar);
    lbI2CErrors->setText("I2C Errors: 0");
    ui->statusBar->insertPermanentWidget(0, lbI2CErrors);

    serial = new QSerialPort(this);

    connect(ui->actionHandleConnection, SIGNAL(triggered()), this,
            SLOT(HandleSerialConnection()));
    connect(ui->actionRead, SIGNAL(triggered()), this, SLOT(HandleReadSettings()));
    connect(ui->actionSet, SIGNAL(triggered()), this, SLOT(HandleApplySettings()));
    connect(ui->actionSave, SIGNAL(triggered()), this, SLOT(HandleSaveSettings()));
    connect(&timer, SIGNAL(timeout()), this, SLOT(ProcessTimeout()));
    connect(serial, SIGNAL(readyRead()), this, SLOT(ReadSerialData()));
    connect(serial, SIGNAL(error(QSerialPort::SerialPortError)), this,
            SLOT(HandleSerialError(QSerialPort::SerialPortError)));
    connect(ui->pushSensor1AccCalibrate, SIGNAL(clicked()), this, SLOT(HandleAccCalibrate()));
    connect(ui->pushSensor1GyroCalibrate, SIGNAL(clicked()), this, SLOT(HandleGyroCalibrate()));

    FillPortsInfo();

    for(int i = 0; i < 4; i++) {
        ui->comboPitchCommand->addItem(OutputCommands[i].item_name, OutputCommands[i].item_id);
        ui->comboRollCommand->addItem(OutputCommands[i].item_name, OutputCommands[i].item_id);
        ui->comboYawCommand->addItem(OutputCommands[i].item_name, OutputCommands[i].item_id);
    }
    ui->comboPitchCommand->setCurrentIndex(3);
    ui->comboRollCommand->setCurrentIndex(3);
    ui->comboYawCommand->setCurrentIndex(3);

    for(int i = 0; i < 5; i++) {
        ui->comboPitchDeadTime->addItem(OutputDeadTime[i].item_name, OutputDeadTime[i].item_id);
        ui->comboRollDeadTime->addItem(OutputDeadTime[i].item_name, OutputDeadTime[i].item_id);
        ui->comboYawDeadTime->addItem(OutputDeadTime[i].item_name, OutputDeadTime[i].item_id);
    }
    ui->comboPitchDeadTime->setCurrentIndex(4);
    ui->comboRollDeadTime->setCurrentIndex(4);
    ui->comboYawDeadTime->setCurrentIndex(4);

    for(int i = 0; i < 6; i++) {
        ui->comboInputChannelPitch->addItem(InputChannel[i].item_name, InputChannel[i].item_id);
        ui->comboInputChannelRoll->addItem(InputChannel[i].item_name, InputChannel[i].item_id);
        ui->comboInputChannelYaw->addItem(InputChannel[i].item_name, InputChannel[i].item_id);
    }
    ui->comboInputChannelPitch->setCurrentIndex(5);
    ui->comboInputChannelRoll->setCurrentIndex(5);
    ui->comboInputChannelYaw->setCurrentIndex(5);

    for(int i = 0; i < 3; i++) {
        ui->comboInputModePitch->addItem(InputMode[i].item_name, InputMode[i].item_id);
        ui->comboInputModeRoll->addItem(InputMode[i].item_name, InputMode[i].item_id);
        ui->comboInputModeYaw->addItem(InputMode[i].item_name, InputMode[i].item_id);
    }
    ui->comboInputModePitch->setCurrentIndex(0);
    ui->comboInputModeRoll->setCurrentIndex(0);
    ui->comboInputModeYaw->setCurrentIndex(0);

    for(int i = 0; i < 6; i++) {
        ui->comboSensor1AxisTOP->addItem(SensorAxis[i].item_name, SensorAxis[i].item_id);
        ui->comboSensor1AxisRIGHT->addItem(SensorAxis[i].item_name, SensorAxis[i].item_id);
    }
    ui->comboSensor1AxisTOP->setCurrentIndex(2);
    ui->comboSensor1AxisRIGHT->setCurrentIndex(3);

    ui->comboData->addItem(PlotData[0].item_name, PlotData[0].item_id);
    ui->comboData->setCurrentIndex(0);

    ui->plotData->addGraph();
     /* line color red for first graph. */
    ui->plotData->graph(0)->setPen(QPen(Qt::red));
    ui->plotData->addGraph();
    /* line color green for second graph. */
    ui->plotData->graph(1)->setPen(QPen(Qt::green));
    ui->plotData->addGraph();
    /* line color blue for third graph. */
    ui->plotData->graph(2)->setPen(QPen(Qt::blue));

    ui->plotData->xAxis->setTickLabelType(QCPAxis::ltDateTime);
    ui->plotData->xAxis->setDateTimeFormat("hh:mm:ss");
    ui->plotData->xAxis->setAutoTickStep(false);
    ui->plotData->xAxis->setTickStep(2);
    ui->plotData->axisRect()->setupFullAxesBox();

    ui->plotData->yAxis->setLabel("Attitude, deg");

    /* make left and bottom axes transfer their ranges to right and top axes: */
    connect(ui->plotData->xAxis, SIGNAL(rangeChanged(QCPRange)),
            ui->plotData->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->plotData->yAxis, SIGNAL(rangeChanged(QCPRange)),
            ui->plotData->yAxis2, SLOT(setRange(QCPRange)));

    connect(ui->checkDataX, SIGNAL(clicked()), this, SLOT(HandleDataXClicked()));
    connect(ui->checkDataY, SIGNAL(clicked()), this, SLOT(HandleDataYClicked()));
    connect(ui->checkDataZ, SIGNAL(clicked()), this, SLOT(HandleDataZClicked()));
}

/**
 * @brief MainWindow::~MainWindow
 */
MainWindow::~MainWindow()
{
    if (fConnected) {
        HandleSerialConnection();
    }
    delete ui;
}

/**
 * @brief MainWindow::FillPortsInfo
 */
void MainWindow::FillPortsInfo()
{
    cbDevList->clear();
    if (QSerialPortInfo::availablePorts().count() == 0) {
        cbDevList->addItem(tr("Serial port not detected"), "None");
    } else {
        foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
            cbDevList->addItem(info.description() + ' ' + '(' + info.portName() + ')', info.portName());
        }
        cbDevList->setEnabled(true);
        ui->actionHandleConnection->setEnabled(true);
    }
}

/**
 * @brief MainWindow::HandleConnection
 */
void MainWindow::HandleSerialConnection()
{
    if (fConnected) {
        timer.stop();
        serial->close();
        ui->actionHandleConnection->setText(tr("Connect"));
        ui->actionRead->setEnabled(false);
        ui->actionSet->setEnabled(false);
        ui->actionSave->setEnabled(false);
        cbDevList->setEnabled(true);
        ui->statusBar->showMessage(tr("Disconnected from: %1").arg(cbDevList->currentText()));
        fConnected = false;
    } else {
        QString portName = cbDevList->currentData().toString();
        serial->setPortName(portName);
        serial->setBaudRate(57600);
        serial->setDataBits(QSerialPort::Data8);
        serial->setParity(QSerialPort::NoParity);
        serial->setStopBits(QSerialPort::OneStop);
        serial->setFlowControl(QSerialPort::NoFlowControl);
        if (serial->open(QIODevice::ReadWrite)) {
                ui->actionHandleConnection->setText(tr("Disconnect"));
                ui->actionRead->setEnabled(true);
                ui->actionSet->setEnabled(true);
                cbDevList->setEnabled(false);
                ui->statusBar->showMessage(tr("Connected to: %1").arg(cbDevList->currentText()));
                fConnected = true;
                timer.start(100);
        } else {
            QMessageBox::critical(this, tr("Serial Error"), serial->errorString());
            ui->statusBar->showMessage(tr("Open error"));
        }
    }
}

/**
 * @brief MainWindow::SendTelemetryData
 * @param pHdr - pointer to data header structure.
 */
void MainWindow::SendTelemetryData(const PDataHdr pHdr)
{
    serial->write((const char*)pHdr, sizeof(pHdr->cmd_id) + sizeof(pHdr->size));
    if (pHdr->size) {
        serial->write((const char*)pHdr->data, pHdr->size);
        serial->write((const char*)&pHdr->crc, sizeof(pHdr->crc));
    }
}

/**
 * @brief  MainWindow::GetCRC32Checksum
 * @param  pHdr - pointer to data header structure.
 * @return crc32 checksum of zero-padded data buffer.
 */
quint32 MainWindow::GetCRC32Checksum(const PDataHdr pHdr)
{
    size_t crc_length = pHdr->size / sizeof(quint32);
    if (pHdr->size % sizeof(quint32)) {
        crc_length++;
    }
    return crc32((const quint32 *)pHdr->data, crc_length);
}

/**
 * @brief MainWindow::HandleReadSettings
 */
void MainWindow::HandleReadSettings()
{
    /* Get board status data. */
    dataHdr.cmd_id = 'b';
    dataHdr.size   = 0;
    SendTelemetryData(&dataHdr);

    /* Get stabilization settings. */
    dataHdr.cmd_id = 's';
    dataHdr.size   = 0;
    SendTelemetryData(&dataHdr);

    /* Get output settings. */
    dataHdr.cmd_id = 'o';
    dataHdr.size   = 0;
    SendTelemetryData(&dataHdr);

    /* Get input settings. */
    dataHdr.cmd_id = 'p';
    dataHdr.size   = 0;
    SendTelemetryData(&dataHdr);

    /* Get input mode settings. */
    dataHdr.cmd_id = 'm';
    dataHdr.size   = 0;
    SendTelemetryData(&dataHdr);

    /* Get sensor settings. */
    dataHdr.cmd_id = 'd';
    dataHdr.size   = 0;
    SendTelemetryData(&dataHdr);
}

/**
 * @brief MainWindow::HandleApplySettings
 */
void MainWindow::HandleApplySettings()
{
    if (GetStabilizationSettings()) {
        /* Write stabilization settings; */
        qDebug() << "Pitch P:" << PID[0].P << "I:" << PID[0].I << "D:" << PID[0].D;
        qDebug() << "Roll  P:" << PID[1].P << "I:" << PID[1].I << "D:" << PID[1].D;
        qDebug() << "Yaw   P:" << PID[2].P << "I:" << PID[2].I << "D:" << PID[2].D;
        qDebug() << sizeof(PID) << "";

        /* Clean data buffer for zero-padded crc32 checksum calculation. */
        memset((void *)dataBuf, 0, sizeof(dataBuf));
        memcpy((void *)dataBuf, (void *)PID, sizeof(PID));
        dataHdr.cmd_id = 'S';
        dataHdr.size   = sizeof(PID);
        dataHdr.data   = dataBuf;
        dataHdr.crc    = GetCRC32Checksum(&dataHdr);

        SendTelemetryData(&dataHdr);
    }

    if (GetOutputSettings()) {
        /* Write output settings; */
        qDebug() << "Pitch Pwr:" << outSettings[0].power << "NPoles:" << outSettings[0].num_poles \
                 << "Rev:" << outSettings[0].reverse << "Cmd:" << outSettings[0].cmd_id \
                 << "DT:" << outSettings[0].dt_id;
        qDebug() << "Roll  Pwr:" << outSettings[1].power << "NPoles:" << outSettings[1].num_poles \
                 << "Rev:" << outSettings[1].reverse << "Cmd:" << outSettings[1].cmd_id \
                 << "DT:" << outSettings[1].dt_id;
        qDebug() << "Yaw   Pwr:" << outSettings[2].power << "NPoles:" << outSettings[2].num_poles \
                 << "Rev:" << outSettings[2].reverse << "Cmd:" << outSettings[2].cmd_id \
                 << "DT:" << outSettings[2].dt_id;
        qDebug() << sizeof(outSettings) << "";

        /* Clean data buffer for zero-padded crc32 checksum calculation. */
        memset((void *)dataBuf, 0, sizeof(dataBuf));
        memcpy((void *)dataBuf, (void *)outSettings, sizeof(outSettings));
        dataHdr.cmd_id = 'O';
        dataHdr.size   = sizeof(outSettings);
        dataHdr.data   = dataBuf;
        dataHdr.crc    = GetCRC32Checksum(&dataHdr);

        SendTelemetryData(&dataHdr);
    }

    if (GetInputSettings()) {
        /* Write input settings; */
        qDebug() << "Pitch Ch#:" << inSettings[0].channel_id << "Min:" << inSettings[0].min_val \
                 << "Mid:" << inSettings[0].mid_val << "Max:" << inSettings[0].max_val;
        qDebug() << "Roll  Ch#:" << inSettings[1].channel_id << "Min:" << inSettings[1].min_val \
                 << "Mid:" << inSettings[1].mid_val << "Max:" << inSettings[1].max_val;
        qDebug() << "Yaw   Ch#:" << inSettings[2].channel_id << "Min:" << inSettings[2].min_val \
                 << "Mid:" << inSettings[2].mid_val << "Max:" << inSettings[2].max_val;
        qDebug() << sizeof(inSettings) << "";

        /* Clean data buffer for zero-padded crc32 checksum calculation. */
        memset((void *)dataBuf, 0, sizeof(dataBuf));
        memcpy((void *)dataBuf, (void *)inSettings, sizeof(inSettings));
        dataHdr.cmd_id = 'I';
        dataHdr.size   = sizeof(inSettings);
        dataHdr.data   = dataBuf;
        dataHdr.crc    = GetCRC32Checksum(&dataHdr);

        SendTelemetryData(&dataHdr);
    }

    if (GetInputModeSettings()) {
        /* Write input mode settings; */
        qDebug() << "Pitch Mode#:" << modeSettings[0].mode_id << "MinA:" << modeSettings[0].min_angle \
                 << "MaxA:" << modeSettings[0].max_angle << "Speed:" << modeSettings[0].speed \
                 << "Offset:" << modeSettings[0].offset;
        qDebug() << "Roll  Mode#:" << modeSettings[1].mode_id << "MinA:" << modeSettings[1].min_angle \
                 << "MaxA:" << modeSettings[1].max_angle << "Speed:" << modeSettings[1].speed \
                 << "Offset:" << modeSettings[1].offset;
        qDebug() << "Yaw   Mode#:" << modeSettings[2].mode_id << "MinA:" << modeSettings[2].min_angle \
                 << "MaxA:" << modeSettings[2].max_angle << "Speed:" << modeSettings[2].speed \
                 << "Offset:" << modeSettings[2].offset;
        qDebug() << sizeof(modeSettings) << "";

        /* Clean data buffer for zero-padded crc32 checksum calculation. */
        memset((void *)dataBuf, 0, sizeof(dataBuf));
        memcpy((void *)dataBuf, (void *)modeSettings, sizeof(modeSettings));
        dataHdr.cmd_id = 'M';
        dataHdr.size   = sizeof(modeSettings);
        dataHdr.data   = dataBuf;
        dataHdr.crc    = GetCRC32Checksum(&dataHdr);

        SendTelemetryData(&dataHdr);
    }

    if (GetSensorSettings()) {
        /* Write sensor settings; */
        qDebug() << "FRONT id:" << sensorSettings[0].axis_id << "dir:" << sensorSettings[0].axis_dir;
        qDebug() << "RIGHT id:" << sensorSettings[1].axis_id << "dir:" << sensorSettings[1].axis_dir;
        qDebug() << "TOP   id:" << sensorSettings[2].axis_id << "dir:" << sensorSettings[2].axis_dir;

        /* Clean data buffer for zero-padded crc32 checksum calculation. */
        memset((void *)dataBuf, 0, sizeof(dataBuf));
        memcpy((void *)dataBuf, (void *)sensorSettings, sizeof(sensorSettings));
        dataHdr.cmd_id = 'D';
        dataHdr.size   = sizeof(sensorSettings);
        dataHdr.data   = dataBuf;
        dataHdr.crc    = GetCRC32Checksum(&dataHdr);

        SendTelemetryData(&dataHdr);
    }
}

/**
 * @brief MainWindow::HandleSaveSettings
 */
void MainWindow::HandleSaveSettings()
{
    HandleApplySettings();

    /* Save to EEPROM. */
    dataHdr.cmd_id = 'c';
    dataHdr.size   = 0;
    SendTelemetryData(&dataHdr);
}

/**
 * @brief MainWindow::ProcessTimeout
 */
void MainWindow::ProcessTimeout()
{
    /* Get accel data. */
    //dataHdr.cmd_id = 'a';
    //dataHdr.size   = 0;
    //SendTelemetryData(&dataHdr);

    /* Get gyro data. */
    //dataHdr.cmd_id = 'g';
    //dataHdr.size   = 0;
    //SendTelemetryData(&dataHdr);

    /* Get attitude data. */
    dataHdr.cmd_id = 'r';
    dataHdr.size   = 0;
    SendTelemetryData(&dataHdr);

    /* Get input values. */
    dataHdr.cmd_id = 'i';
    dataHdr.size   = 0;
    SendTelemetryData(&dataHdr);

    /* Get offset values. */
    dataHdr.cmd_id = 'h';
    dataHdr.size   = 0;
    SendTelemetryData(&dataHdr);

    //lbI2CErrors->setText(tr("I2C Errors: %1").arg(nCounter));
}

/**
 * @brief MainWindow::ReadSerialData
 */
void MainWindow::ReadSerialData()
{
    qint64 bytesAvailable = serial->bytesAvailable();

    if (bytesRequired) { /* Continue with previous command. */
        if (bytesAvailable >= bytesRequired) {
            if (dataHdr.size % sizeof(quint32)) {
                /* Clean data buffer for zero-padded crc32 checksum calculation. */
                memset((void *)dataBuf, 0, sizeof(dataBuf));
            }
            serial->read(dataBuf, dataHdr.size);
            serial->read((char *)&dataHdr.crc, sizeof(dataHdr.crc));
            dataHdr.data = dataBuf;
            bytesRequired = 0;
            ProcessSerialCommands(&dataHdr);
            bytesAvailable -= dataHdr.size + sizeof(dataHdr.crc);
            if (bytesAvailable) {
                ReadSerialData();
            }
        }
    } else { /* Read next command from the queue. */
        if (bytesAvailable >= 2) {
            serial->read((char *)&dataHdr, 2);
            bytesAvailable -= 2;
            if (dataHdr.size) {
                if (bytesAvailable >= (dataHdr.size + sizeof(dataHdr.crc))) {
                    if (dataHdr.size % sizeof(quint32)) {
                        /* Clean data buffer for zero-padded crc32 checksum calculation. */
                        memset((void *)dataBuf, 0, sizeof(dataBuf));
                    }
                    serial->read(dataBuf, dataHdr.size);
                    serial->read((char *)&dataHdr.crc, sizeof(dataHdr.crc));
                    dataHdr.data = dataBuf;
                    ProcessSerialCommands(&dataHdr);
                    bytesAvailable -= dataHdr.size + sizeof(dataHdr.crc);
                    if (bytesAvailable) {
                        ReadSerialData();
                    }
                } else {
                    bytesRequired = dataHdr.size + sizeof(dataHdr.crc);
                }
            } else {
                ProcessSerialCommands(&dataHdr);
                if (bytesAvailable) {
                    ReadSerialData();
                }
            }
        }
    }
}

/**
 * @brief MainWindow::HandleSerialError
 * @param error
 */
void MainWindow::HandleSerialError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ResourceError) {
        if (fConnected ) {
            HandleSerialConnection();
        }
        QMessageBox::critical(this, tr("Critical Error"), serial->errorString());
    }
}

/**
 * @brief MainWindow::HandleDataXClicked
 */
void MainWindow::HandleDataXClicked()
{
    ui->plotData->graph(0)->setVisible(ui->checkDataX->isChecked());
}

/**
 * @brief MainWindow::HandleDataYClicked
 */
void MainWindow::HandleDataYClicked()
{
    ui->plotData->graph(1)->setVisible(ui->checkDataY->isChecked());
}

/**
 * @brief MainWindow::HandleDataZClicked
 */
void MainWindow::HandleDataZClicked()
{
    ui->plotData->graph(2)->setVisible(ui->checkDataZ->isChecked());
}

void MainWindow::HandleAccCalibrate()
{
    /* Start accel calibration. */
    dataHdr.cmd_id = ']';
    dataHdr.size   = 0;
    SendTelemetryData(&dataHdr);
}

void MainWindow::HandleGyroCalibrate()
{
    /* Start gyro calibration. */
    dataHdr.cmd_id = '[';
    dataHdr.size   = 0;
    SendTelemetryData(&dataHdr);
}

/**
 * @brief MainWindow::ProcessSerialCommands
 * @param pHdr
 */
void MainWindow::ProcessSerialCommands(const PDataHdr pHdr)
{
    float rpy[3];
    float *pfloatBuf;
    QQuaternion attiQ;
    QQuaternion diffQ;
    bool fEnlarge = false;
    double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;

    switch (pHdr->cmd_id) {
    case 'a':
        if ((pHdr->size == sizeof(float) * 3) && (pHdr->crc == GetCRC32Checksum(pHdr))) {
            pfloatBuf = (float *)(pHdr->data);
        } else {
            qDebug() << "Acc CRC32 mismatch:" << hex << pHdr->crc << "|" << GetCRC32Checksum(pHdr);
        }
        break;
    case 'b':
        if ((pHdr->size == sizeof(boardStatus)) && (pHdr->crc == GetCRC32Checksum(pHdr))) {
            boardStatus = *(pHdr->data);
            /* Check if sensor detected. */
            if (boardStatus & 1) {
                ui->groupSensor1->setTitle("Sensor1 (MPU6050):");
                ui->comboSensor1AxisTOP->setEnabled(true);
                ui->comboSensor1AxisRIGHT->setEnabled(true);
                ui->pushSensor1AccCalibrate->setEnabled(true);
                ui->pushSensor1GyroCalibrate->setEnabled(true);
            } else {
                ui->groupSensor1->setTitle("Sensor1 (none):");
                ui->comboSensor1AxisTOP->setEnabled(false);
                ui->comboSensor1AxisRIGHT->setEnabled(false);
                ui->pushSensor1AccCalibrate->setEnabled(false);
                ui->pushSensor1GyroCalibrate->setEnabled(false);
            }
            /* Check if EEPROM detected. */
            if (boardStatus & 2) {
                ui->actionSave->setEnabled(true);
            } else {
                ui->actionSave->setEnabled(false);
            }
            qDebug() << "Board status:\r\n  MPU6050 detected:" << ((boardStatus & 1) == 1) \
                     << "\r\n  EEPROM detected:" << ((boardStatus & 2) == 2);
        } else {
            qDebug() << "Board status CRC32 mismatch:" << hex << pHdr->crc << "|" << GetCRC32Checksum(pHdr);
        }
        break;
    case 'd':
        if ((pHdr->size == sizeof(sensorSettings)) && (pHdr->crc == GetCRC32Checksum(pHdr))) {
            memcpy((void *)sensorSettings, (void *)pHdr->data, pHdr->size);
            SetSensorSettings();
        } else {
            qDebug() << "Sensor settings CRC32 mismatch:" << hex << pHdr->crc << "|" << GetCRC32Checksum(pHdr);
        }
        break;
    case 'g':
        if ((pHdr->size == sizeof(float) * 3) && (pHdr->crc == GetCRC32Checksum(pHdr))) {
            pfloatBuf = (float *)(pHdr->data);
        } else {
            qDebug() << "Gyro CRC32 mismatch:" << hex << pHdr->crc << "|" << GetCRC32Checksum(pHdr);
        }
        break;
    case 'h':
        if ((pHdr->size == sizeof(float) * 3) && (pHdr->crc == GetCRC32Checksum(pHdr))) {
            pfloatBuf = (float *)(pHdr->data);
            ui->labelOffsetPitch->setText(tr("%1").arg(round(pfloatBuf[0]*RAD2DEG/(outSettings[0].num_poles >> 1))));
            ui->labelOffsetRoll->setText(tr("%1").arg(round(pfloatBuf[1]*RAD2DEG/(outSettings[1].num_poles >> 1))));
            ui->labelOffsetYaw->setText(tr("%1").arg(round(pfloatBuf[2]*RAD2DEG/(outSettings[2].num_poles >> 1))));
        } else {
            qDebug() << "Motor offset CRC32 mismatch:" << hex << pHdr->crc << "|" << GetCRC32Checksum(pHdr);
        }
        break;
    case 'i':
        if ((pHdr->size == sizeof(inputValues)) && (pHdr->crc == GetCRC32Checksum(pHdr))) {
            memcpy((void *)inputValues, (void *)pHdr->data, pHdr->size);
            if (inSettings[0].channel_id < 5) {
                ui->labelInputPitch->setText(tr("%1").arg(inputValues[inSettings[0].channel_id]));
            } else {
                ui->labelInputPitch->setText(tr("%1").arg(inSettings[0].min_val));
            }
            if (inSettings[1].channel_id < 5) {
                ui->labelInputRoll->setText(tr("%1").arg(inputValues[inSettings[1].channel_id]));
            } else {
                ui->labelInputRoll->setText(tr("%1").arg(inSettings[1].min_val));
            }
            if (inSettings[2].channel_id < 5) {
                ui->labelInputYaw->setText(tr("%1").arg(inputValues[inSettings[2].channel_id]));
            } else {
                ui->labelInputYaw->setText(tr("%1").arg(inSettings[2].min_val));
            }
        } else {
            qDebug() << "Input values CRC32 mismatch:" << hex << pHdr->crc << "|" << GetCRC32Checksum(pHdr);
        }
        break;
    case 'm':
        if ((pHdr->size == sizeof(modeSettings)) && (pHdr->crc == GetCRC32Checksum(pHdr))) {
            memcpy((void *)modeSettings, (void *)pHdr->data, pHdr->size);
            SetInputModeSettings();
        } else {
            qDebug() << "Input mode settings CRC32 mismatch:" << hex << pHdr->crc << "|" << GetCRC32Checksum(pHdr);
        }
        break;
    case 'o':
        if ((pHdr->size == sizeof(outSettings)) && (pHdr->crc == GetCRC32Checksum(pHdr))) {
            memcpy((void *)outSettings, (void *)pHdr->data, pHdr->size);
            SetOutputSettings();
        } else {
            qDebug() << "Output settings CRC32 mismatch:" << hex << pHdr->crc << "|" << GetCRC32Checksum(pHdr);
        }
        break;
    case 'p':
        if ((pHdr->size == sizeof(inSettings)) && (pHdr->crc == GetCRC32Checksum(pHdr))) {
            memcpy((void *)inSettings, (void *)pHdr->data, pHdr->size);
            SetInputSettings();
        } else {
            qDebug() << "Input settings CRC32 mismatch:" << hex << pHdr->crc << "|" << GetCRC32Checksum(pHdr)
                     << "Size:" << dec << pHdr->size << "|" << sizeof(inSettings);
        }
        break;
    case 'r':
        if ((pHdr->size == sizeof(float) * 4) && (pHdr->crc == GetCRC32Checksum(pHdr))) {
            pfloatBuf = (float *)(pHdr->data);

            Quaternion2RPY(pfloatBuf, rpy);

            /*
             * Construct an attitude quaternion.
             * WARNING! OpenGL and IMU coordinate systems do not match.
             * Adjust indexes and direction of rotations of IMU data to match
             * OpenGL coordinate system.
             */
            attiQ = QQuaternion(pfloatBuf[0], pfloatBuf[1], -pfloatBuf[3], -pfloatBuf[2]);

            /*
             * Find the difference between last attitude and current attitude.
             * diffQ is a rotation quaternion that transforms lastQ into attiQ.
             * Because attiQ is normalized quaternion, the inverse of attiQ is equal to
             * conjugate of attiQ.
             * NOTE! The order of multiplication IS IMPORTANT!
            */
            diffQ = attiQ.conjugate() * lastQ;
            diffQ.normalize();

            lastQ = attiQ;

            ui->plotData->graph(0)->setVisible(ui->checkDataX->isChecked());
            ui->plotData->graph(1)->setVisible(ui->checkDataY->isChecked());
            ui->plotData->graph(2)->setVisible(ui->checkDataZ->isChecked());
            // add new data:
            ui->plotData->graph(0)->addData(key, rpy[0] * RAD2DEG);
            ui->plotData->graph(1)->addData(key, rpy[1] * RAD2DEG);
            ui->plotData->graph(2)->addData(key, rpy[2] * RAD2DEG);
            // remove data of lines that's outside visible range:
            ui->plotData->graph(0)->removeDataBefore(key-8);
            ui->plotData->graph(1)->removeDataBefore(key-8);
            ui->plotData->graph(2)->removeDataBefore(key-8);
            // rescale value (vertical) axis to fit the current data:
            if (ui->plotData->graph(0)->visible()) {
                ui->plotData->graph(0)->rescaleValueAxis();
                fEnlarge = true;
            }
            if (ui->plotData->graph(1)->visible()) {
                ui->plotData->graph(1)->rescaleValueAxis(fEnlarge);
                fEnlarge = true;
            }
            if (ui->plotData->graph(2)->visible()) {
                ui->plotData->graph(2)->rescaleValueAxis(fEnlarge);
            }

            ui->plotData->xAxis->setRange(key+0.25, 8, Qt::AlignRight);
            ui->plotData->replot();

            ui->widget->rotateBy(&diffQ);
        } else {
            qDebug() << "RPY CRC32 mismatch:" << hex << pHdr->crc << "|" << GetCRC32Checksum(pHdr);
        }
        break;
    case 's':
        if ((pHdr->size == sizeof(PID)) && (pHdr->crc == GetCRC32Checksum(pHdr))) {
            memcpy((void *)PID, (void *)pHdr->data, pHdr->size);
            SetStabilizationSettings();
        } else {
            qDebug() << "PID settings CRC32 mismatch:" << hex << pHdr->crc << "|" << GetCRC32Checksum(pHdr);
        }
        break;
    default:;
    }
}

/**
 * @brief MainWindow::GetStabilizationSettings
 * @return
 */
bool MainWindow::GetStabilizationSettings()
{
    bool fUpdated = false;
    quint8 temp;

    /* PITCH */
    temp = ui->spinPitch_P->value();
    if (temp != PID[0].P) {
        PID[0].P = temp;
        fUpdated = true;
    }
    temp = ui->spinPitch_I->value();
    if (temp != PID[0].I) {
        PID[0].I = temp;
        fUpdated = true;
    }
    temp = ui->spinPitch_D->value();
    if (temp != PID[0].D) {
        PID[0].D = temp;
        fUpdated = true;
    }

    /* ROLL */
    temp = ui->spinRoll_P->value();
    if (temp != PID[1].P) {
        PID[1].P = temp;
        fUpdated = true;
    }
    temp = ui->spinRoll_I->value();
    if (temp != PID[1].I) {
        PID[1].I = temp;
        fUpdated = true;
    }
    temp = ui->spinRoll_D->value();
    if (temp != PID[1].D) {
        PID[1].D = temp;
        fUpdated = true;
    }

    /* YAW */
    temp = ui->spinYaw_P->value();
    if (temp != PID[2].P) {
        PID[2].P = temp;
        fUpdated = true;
    }
    temp = ui->spinYaw_I->value();
    if (temp != PID[2].I) {
        PID[2].I = temp;
        fUpdated = true;
    }
    temp = ui->spinYaw_D->value();
    if (temp != PID[2].D) {
        PID[2].D = temp;
        fUpdated = true;
    }

    return fUpdated;
}

/**
 * @brief MainWindow::SetStabilizationSettings
 * @return
 */
bool MainWindow::SetStabilizationSettings()
{
    /* PITCH */
    ui->spinPitch_P->setValue(PID[0].P);
    ui->spinPitch_I->setValue(PID[0].I);
    ui->spinPitch_D->setValue(PID[0].D);
    /* ROLL */
    ui->spinRoll_P->setValue(PID[1].P);
    ui->spinRoll_I->setValue(PID[1].I);
    ui->spinRoll_D->setValue(PID[1].D);
    /* YAW */
    ui->spinYaw_P->setValue(PID[2].P);
    ui->spinYaw_I->setValue(PID[2].I);
    ui->spinYaw_D->setValue(PID[2].D);
    return true;
}

/**
 * @brief MainWindow::GetOutputSettings
 * @return
 */
bool MainWindow::GetOutputSettings()
{
    bool fUpdated = false;
    quint8 temp;

    /* PITCH */
    temp = ui->spinPitchPower->value();
    if (temp != outSettings[0].power) {
        outSettings[0].power = temp;
        fUpdated = true;
    }
    temp = ui->spinPitchNumPoles->value();
    if (temp != outSettings[0].num_poles) {
        outSettings[0].num_poles = temp;
        fUpdated = true;
    }
    temp = ui->checkPitchRev->isChecked();
    if (temp != outSettings[0].reverse) {
        outSettings[0].reverse = temp;
        fUpdated = true;
    }
    temp = ui->comboPitchCommand->currentIndex();
    if (temp != outSettings[0].cmd_id) {
        outSettings[0].cmd_id = temp;
        fUpdated = true;
    }
    temp = ui->comboPitchDeadTime->currentIndex();
    if (temp != outSettings[0].dt_id) {
        outSettings[0].dt_id = temp;
        fUpdated = true;
    }

    /* ROLL */
    temp = ui->spinRollPower->value();
    if (temp != outSettings[1].power) {
        outSettings[1].power = temp;
        fUpdated = true;
    }
    temp = ui->spinRollNumPoles->value();
    if (temp != outSettings[1].num_poles) {
        outSettings[1].num_poles = temp;
        fUpdated = true;
    }
    temp = ui->checkRollRev->isChecked();
    if (temp != outSettings[1].reverse) {
        outSettings[1].reverse = temp;
        fUpdated = true;
    }
    temp = ui->comboRollCommand->currentIndex();
    if (temp != outSettings[1].cmd_id) {
        outSettings[1].cmd_id = temp;
        fUpdated = true;
    }
    temp = ui->comboRollDeadTime->currentIndex();
    if (temp != outSettings[1].dt_id) {
        outSettings[1].dt_id = temp;
        fUpdated = true;
    }

    /* YAW */
    temp = ui->spinYawPower->value();
    if (temp != outSettings[2].power) {
        outSettings[2].power = temp;
        fUpdated = true;
    }
    temp = ui->spinYawNumPoles->value();
    if (temp != outSettings[2].num_poles) {
        outSettings[2].num_poles = temp;
        fUpdated = true;
    }
    temp = ui->checkYawRev->isChecked();
    if (temp != outSettings[2].reverse) {
        outSettings[2].reverse = temp;
        fUpdated = true;
    }
    temp = ui->comboYawCommand->currentIndex();
    if (temp != outSettings[2].cmd_id) {
        outSettings[2].cmd_id = temp;
        fUpdated = true;
    }
    temp = ui->comboYawDeadTime->currentIndex();
    if (temp != outSettings[2].dt_id) {
        outSettings[2].dt_id = temp;
        fUpdated = true;
    }
    return fUpdated;
}

/**
 * @brief MainWindow::SetOutputSettings
 * @return
 */
bool MainWindow::SetOutputSettings()
{
    /* PITCH */
    ui->spinPitchPower->setValue(outSettings[0].power);
    ui->spinPitchNumPoles->setValue(outSettings[0].num_poles);
    ui->checkPitchRev->setChecked(outSettings[0].reverse > 0);
    ui->comboPitchCommand->setCurrentIndex(outSettings[0].cmd_id);
    ui->comboPitchDeadTime->setCurrentIndex(outSettings[0].dt_id);
    /* ROLL */
    ui->spinRollPower->setValue(outSettings[1].power);
    ui->spinRollNumPoles->setValue(outSettings[1].num_poles);
    ui->checkRollRev->setChecked(outSettings[1].reverse > 0);
    ui->comboRollCommand->setCurrentIndex(outSettings[1].cmd_id);
    ui->comboRollDeadTime->setCurrentIndex(outSettings[1].dt_id);
    /* YAW */
    ui->spinYawPower->setValue(outSettings[2].power);
    ui->spinYawNumPoles->setValue(outSettings[2].num_poles);
    ui->checkYawRev->setChecked(outSettings[2].reverse > 0);
    ui->comboYawCommand->setCurrentIndex(outSettings[2].cmd_id);
    ui->comboYawDeadTime->setCurrentIndex(outSettings[2].dt_id);
    return true;
}

/**
 * @brief MainWindow::GetInputSettings
 * @return
 */
bool MainWindow::GetInputSettings()
{
    bool fUpdated = false;
    quint16 temp;

    /* PITCH */
    temp = ui->comboInputChannelPitch->currentIndex();
    if (temp != inSettings[0].channel_id) {
        inSettings[0].channel_id = (quint8)temp;
        fUpdated = true;
    }
    temp = ui->spinInputMinPitch->value();
    if (temp != inSettings[0].min_val) {
        inSettings[0].min_val = temp;
        fUpdated = true;
    }
    temp = ui->spinInputMidPitch->value();
    if (temp != inSettings[0].mid_val) {
        inSettings[0].mid_val = temp;
        fUpdated = true;
    }
    temp = ui->spinInputMaxPitch->value();
    if (temp != inSettings[0].max_val) {
        inSettings[0].max_val = temp;
        fUpdated = true;
    }

    /* ROLL */
    temp = ui->comboInputChannelRoll->currentIndex();
    if (temp != inSettings[1].channel_id) {
        inSettings[1].channel_id = (quint8)temp;
        fUpdated = true;
    }
    temp = ui->spinInputMinRoll->value();
    if (temp != inSettings[1].min_val) {
        inSettings[1].min_val = temp;
        fUpdated = true;
    }
    temp = ui->spinInputMidRoll->value();
    if (temp != inSettings[1].mid_val) {
        inSettings[1].mid_val = temp;
        fUpdated = true;
    }
    temp = ui->spinInputMaxRoll->value();
    if (temp != inSettings[1].max_val) {
        inSettings[1].max_val = temp;
        fUpdated = true;
    }

    /* YAW */
    temp = ui->comboInputChannelYaw->currentIndex();
    if (temp != inSettings[2].channel_id) {
        inSettings[2].channel_id = (quint8)temp;
        fUpdated = true;
    }
    temp = ui->spinInputMinYaw->value();
    if (temp != inSettings[2].min_val) {
        inSettings[2].min_val = temp;
        fUpdated = true;
    }
    temp = ui->spinInputMidYaw->value();
    if (temp != inSettings[2].mid_val) {
        inSettings[2].mid_val = temp;
        fUpdated = true;
    }
    temp = ui->spinInputMaxYaw->value();
    if (temp != inSettings[2].max_val) {
        inSettings[2].max_val = temp;
        fUpdated = true;
    }
    return fUpdated;
}

/**
 * @brief MainWindow::SetInputSettings
 * @return
 */
bool MainWindow::SetInputSettings()
{
    /* PITCH */
    ui->comboInputChannelPitch->setCurrentIndex(inSettings[0].channel_id);
    ui->spinInputMinPitch->setValue(inSettings[0].min_val);
    ui->spinInputMidPitch->setValue(inSettings[0].mid_val);
    ui->spinInputMaxPitch->setValue(inSettings[0].max_val);
    /* ROLL */
    ui->comboInputChannelRoll->setCurrentIndex(inSettings[1].channel_id);
    ui->spinInputMinRoll->setValue(inSettings[1].min_val);
    ui->spinInputMidRoll->setValue(inSettings[1].mid_val);
    ui->spinInputMaxRoll->setValue(inSettings[1].max_val);
    /* YAW */
    ui->comboInputChannelYaw->setCurrentIndex(inSettings[2].channel_id);
    ui->spinInputMinYaw->setValue(inSettings[2].min_val);
    ui->spinInputMidYaw->setValue(inSettings[2].mid_val);
    ui->spinInputMaxYaw->setValue(inSettings[2].max_val);
    return true;
}

/**
 * @brief MainWindow::GetInputModeSettings
 * @return
 */
bool MainWindow::GetInputModeSettings()
{
    bool fUpdated = false;
    qint16 temp;

    /* PITCH */
    temp = ui->comboInputModePitch->currentIndex();
    if (temp != modeSettings[0].mode_id) {
        modeSettings[0].mode_id = (quint8)temp;
        fUpdated = true;
    }
    temp = ui->spinInputMinAnglePitch->value();
    if (temp != modeSettings[0].min_angle) {
        modeSettings[0].min_angle = temp;
        fUpdated = true;
    }
    temp = ui->spinInputMaxAnglePitch->value();
    if (temp != modeSettings[0].max_angle) {
        modeSettings[0].max_angle = temp;
        fUpdated = true;
    }
    temp = ui->spinInputSpeedPitch->value();
    if (temp != modeSettings[0].speed) {
        modeSettings[0].speed = temp;
        fUpdated = true;
    }
    temp = ui->spinInputOffsetPitch->value();
    if (temp != modeSettings[0].offset) {
        modeSettings[0].offset = temp;
        fUpdated = true;
    }

    /* ROLL */
    temp = ui->comboInputModeRoll->currentIndex();
    if (temp != modeSettings[1].mode_id) {
        modeSettings[1].mode_id = (quint8)temp;
        fUpdated = true;
    }
    temp = ui->spinInputMinAngleRoll->value();
    if (temp != modeSettings[1].min_angle) {
        modeSettings[1].min_angle = temp;
        fUpdated = true;
    }
    temp = ui->spinInputMaxAngleRoll->value();
    if (temp != modeSettings[1].max_angle) {
        modeSettings[1].max_angle = temp;
        fUpdated = true;
    }
    temp = ui->spinInputSpeedRoll->value();
    if (temp != modeSettings[1].speed) {
        modeSettings[1].speed = temp;
        fUpdated = true;
    }
    temp = ui->spinInputOffsetRoll->value();
    if (temp != modeSettings[1].offset) {
        modeSettings[1].offset = temp;
        fUpdated = true;
    }

    /* YAW */
    temp = ui->comboInputModeYaw->currentIndex();
    if (temp != modeSettings[2].mode_id) {
        modeSettings[2].mode_id = (quint8)temp;
        fUpdated = true;
    }
    temp = ui->spinInputMinAngleYaw->value();
    if (temp != modeSettings[2].min_angle) {
        modeSettings[2].min_angle = temp;
        fUpdated = true;
    }
    temp = ui->spinInputMaxAngleYaw->value();
    if (temp != modeSettings[2].max_angle) {
        modeSettings[2].max_angle = temp;
        fUpdated = true;
    }
    temp = ui->spinInputSpeedYaw->value();
    if (temp != modeSettings[2].speed) {
        modeSettings[2].speed = temp;
        fUpdated = true;
    }
    temp = ui->spinInputOffsetYaw->value();
    if (temp != modeSettings[2].offset) {
        modeSettings[2].offset = temp;
        fUpdated = true;
    }
    return fUpdated;
}

/**
 * @brief MainWindow::SetInputModeSettings
 * @return
 */
bool MainWindow::SetInputModeSettings()
{
    /* PITCH */
    ui->comboInputModePitch->setCurrentIndex(modeSettings[0].mode_id);
    ui->spinInputMinAnglePitch->setValue(modeSettings[0].min_angle);
    ui->spinInputMaxAnglePitch->setValue(modeSettings[0].max_angle);
    ui->spinInputSpeedPitch->setValue(modeSettings[0].speed);
    ui->spinInputOffsetPitch->setValue(modeSettings[0].offset);
    /* ROLL */
    ui->comboInputModeRoll->setCurrentIndex(modeSettings[1].mode_id);
    ui->spinInputMinAngleRoll->setValue(modeSettings[1].min_angle);
    ui->spinInputMaxAngleRoll->setValue(modeSettings[1].max_angle);
    ui->spinInputSpeedRoll->setValue(modeSettings[1].speed);
    ui->spinInputOffsetRoll->setValue(modeSettings[1].offset);
    /* YAW */
    ui->comboInputModeYaw->setCurrentIndex(modeSettings[2].mode_id);
    ui->spinInputMinAngleYaw->setValue(modeSettings[2].min_angle);
    ui->spinInputMaxAngleYaw->setValue(modeSettings[2].max_angle);
    ui->spinInputSpeedYaw->setValue(modeSettings[2].speed);
    ui->spinInputOffsetYaw->setValue(modeSettings[2].offset);
    return true;
}

/**
 * @brief MainWindow::GetSensorSettings
 * @return
 */
bool MainWindow::GetSensorSettings()
{
    bool fUpdated = false;
    quint16 temp;

    /* TOP axis */
    temp = ui->comboSensor1AxisTOP->currentIndex();
    if (temp != sensorAxes[2]) {
        sensorAxes[2] = (quint8)temp;
        fUpdated = true;
    }
    /* RIGHT axis */
    temp = ui->comboSensor1AxisRIGHT->currentIndex();
    if (temp != sensorAxes[0]) {
        sensorAxes[0] = (quint8)temp;
        fUpdated = true;
    }
    /* FRONT axis */
    if (fUpdated) {
        sensorAxes[1] = amMtx[sensorAxes[0]][sensorAxes[2]];
        if (sensorAxes[1] == 6) {
            QMessageBox::critical(this, tr("Axes error"),
                "Top and right axes of the sensor cannot be the same!");
            return false;
        }
    }

    for (quint8 i = 0; i < 3; i++) {
        switch (sensorAxes[i]) {
        case 0: // +X;
            sensorSettings[i].axis_id = 0;
            sensorSettings[i].axis_dir = -1;
            break;
        case 1: // +Y;
            sensorSettings[i].axis_id = 1;
            sensorSettings[i].axis_dir = 1;
            break;
        case 2: // +Z;
            sensorSettings[i].axis_id = 2;
            sensorSettings[i].axis_dir = -1;
            break;
        case 3: // -X;
            sensorSettings[i].axis_id = 0;
            sensorSettings[i].axis_dir = 1;
            break;
        case 4: // -Y;
            sensorSettings[i].axis_id = 1;
            sensorSettings[i].axis_dir = -1;
            break;
        case 5: // -Z;
            sensorSettings[i].axis_id = 2;
            sensorSettings[i].axis_dir = 1;
            break;
        }
    }

    return fUpdated;
}

/**
 * @brief MainWindow::SetSensorSettings
 * @return
 */
bool MainWindow::SetSensorSettings()
{
    for (quint8 i = 0; i < 3; i++) {
        switch (sensorSettings[i].axis_id) {
        case 0: // X;
            if (sensorSettings[i].axis_dir > 0) {
                sensorAxes[i] = 3;
            } else {
                sensorAxes[i] = 0;
            }
            break;
        case 1: // Y;
            if (sensorSettings[i].axis_dir > 0) {
                sensorAxes[i] = 1;
            } else {
                sensorAxes[i] = 4;
            }
            break;
        case 2: // Z;
            if (sensorSettings[i].axis_dir > 0) {
                sensorAxes[i] = 5;
            } else {
                sensorAxes[i] = 2;
            }
            break;
        }
    }
    ui->comboSensor1AxisTOP->setCurrentIndex(sensorAxes[2]);
    ui->comboSensor1AxisRIGHT->setCurrentIndex(sensorAxes[0]);
    return true;
}
