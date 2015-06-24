#ifndef MAVLINK_HANDLER_H_
#define MAVLINK_HANDLER_H_

#define MAX_STREAM_RATE_HZ 50

typedef enum {
  STREAM_EXTRA1,
  STREAM_RC,
  STREAM_RAW_IMU,
  STREAM_HEARTHBEAT,
  NUM_STREAMS
} streams;

#ifdef __cplusplus
extern "C" {
#endif
void readMavlinkChannel(void);
void handleStream(void);
#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_HANDLER_H_ */
