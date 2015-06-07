#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <QtGlobal>

#define TELEMETRY_BUFFER_SIZE           0x80

#define TELEMETRY_MSG_SOF_ID            0x00
#define TELEMETRY_MSG_SIZE_ID           0x02
#define TELEMETRY_HDR_SIZE_BYTES        0x04
#define TELEMETRY_CRC_SIZE_BYTES        0x04
/* TODO: unify naming with FW */
#define TELEMETRY_MSG_SIZE_BYTES        ( TELEMETRY_HDR_SIZE_BYTES + TELEMETRY_CRC_SIZE_BYTES )
#define TELEMETRY_MSG_SIZE_BYTES_MAX    0x80

#define TELEMETRY_MSG_SOF               0xBD

/* Predefined telemetry responses. */
#define TELEMETRY_RESP_OK         "_OK_"
#define TELEMETRY_RESP_FAIL       "FAIL"

typedef struct tagTelemetryMessage {
    quint8 sof;     /* Start-of-frame ID */
    quint8 msg_id;  /* Message ID. */
    quint8 size;    /* Size of whole telemetry message including header and data in bytes. */
    quint8 res;     /* Reserved. Set to 0. */
    char data[TELEMETRY_BUFFER_SIZE];
    quint32 crc;    /* CRC32 checksum. */
} __attribute__((packed)) TelemetryMessage, *PTelemetryMessage;

Q_DECLARE_METATYPE(TelemetryMessage);

#endif // TELEMETRY_H
