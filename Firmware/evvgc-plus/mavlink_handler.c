#include "ch.h"
#include "hal.h"

#include "attitude.h"
#include "mpu6050.h"
#include "misc.h"
#include "mavlink.h"

#include "mavlink_handler.h"

static mavlink_system_t mavlink_sys = { 20, MAV_COMP_ID_GIMBAL };

static uint8_t system_type = MAV_TYPE_GIMBAL;
static uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

static uint8_t system_mode = MAV_MODE_TEST_ARMED;
static uint32_t custom_mode = 0; ///< Custom mode, can be defined by user/adopter
static uint8_t system_state = MAV_STATE_ACTIVE;

static mavlink_message_t msg;
static mavlink_status_t status;

static uint8_t mavBuff[64];

static uint8_t streamRates[NUM_STREAMS] = {0};
static uint8_t stream_tics[NUM_STREAMS] = {255};

bool_t stream_trigger(streams stream_num) {
  if (stream_num >= NUM_STREAMS) {
    return 0;
  }

  uint8_t rate = streamRates[stream_num];

  if(rate == 0) {
    return 0;
  }
  if(stream_tics[stream_num] == 0) {
    if(rate > MAX_STREAM_RATE_HZ) {
      rate = MAX_STREAM_RATE_HZ;
    }
    stream_tics[stream_num] = (MAX_STREAM_RATE_HZ / rate) - 1;
    return 1;
  }
  stream_tics[stream_num]--;
  return 0;
}

void handle_requset_data_stream(mavlink_message_t* reqmsg) {
  mavlink_request_data_stream_t packet;
  mavlink_msg_request_data_stream_decode(reqmsg, &packet);

  uint8_t freq;

  if(packet.start_stop == 0) {
    freq = 0;
  }
  else if(packet.start_stop == 1) {
    freq = packet.req_message_rate;
  }
  else return;

  switch(packet.req_stream_id) {
    case MAV_DATA_STREAM_ALL: {
      uint8_t i = 0;
      for(i = 0; i < STREAM_HEARTHBEAT; i++) {
        streamRates[i] = freq;
      }
      break;
    }
    case MAV_DATA_STREAM_EXTRA1:
      streamRates[STREAM_EXTRA1] = freq;
      break;
    case MAV_DATA_STREAM_RC_CHANNELS:
      streamRates[STREAM_RC] = freq;
      break;
    case MAV_DATA_STREAM_RAW_SENSORS:
      streamRates[STREAM_RAW_IMU] = freq;
      break;
  }
}

void handleStream(void) {
  mavlink_message_t msg_s;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  streamRates[STREAM_HEARTHBEAT] = 1;

  if(stream_trigger(STREAM_HEARTHBEAT)){
    mavlink_msg_heartbeat_pack(mavlink_sys.sysid, mavlink_sys.compid, &msg_s,
      system_type, autopilot_type, system_mode, custom_mode, system_state);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_s);
    chnWrite(&SD4, buf, len);
  }

  if(stream_trigger(STREAM_EXTRA1)) {
    float RPY[3];
    Quaternion2RPY(g_IMU1.rpy, RPY);
    mavlink_msg_attitude_pack(mavlink_sys.sysid, mavlink_sys.compid, &msg_s, MS2ST(chTimeNow()),
        g_IMU1.rpy[1], g_IMU1.rpy[0], g_IMU1.rpy[2], .0f, .0f, .0f);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_s);
    chnWrite(&SD4, buf, len);
  }

  if(stream_trigger(STREAM_RC)) {

  }

  if(stream_trigger(STREAM_RAW_IMU)) {
    //TODO: Have to output RAW data.
    mavlink_msg_raw_imu_pack(mavlink_sys.sysid, mavlink_sys.compid, &msg_s, MS2ST(chTimeNow()),
        (int16_t)g_IMU1.accelData[0], (int16_t)g_IMU1.accelData[1], (int16_t)g_IMU1.accelData[2] ,
        (int16_t)g_IMU1.gyroData[0], (int16_t)g_IMU1.gyroData[1], (int16_t)g_IMU1.gyroData[2],
        0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_s);
    chnWrite(&SD4, buf, len);
  }
}

void readMavlinkChannel(void) {

  uint8_t bytesRead = chnReadTimeout(&SD4, mavBuff, 64, MS2ST(5));

  uint8_t i;
  for(i = 0; i < bytesRead; i++) {
    if (mavlink_parse_char(MAVLINK_COMM_0, mavBuff[i], &msg, &status)) {
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
          break;
        }
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
          handle_requset_data_stream(&msg);
          break;
        }
        case MAVLINK_MSG_ID_COMMAND_LONG: {
          mavlink_command_long_t dec;
          mavlink_msg_command_long_decode(&msg, &dec);
          switch (dec.command) {
            case MAV_CMD_DO_MOUNT_CONTROL: {
              if (dec.param7 == MAV_MOUNT_MODE_NEUTRAL)
                setCameraRotation(0.0f, 0.0f, 0.0f);
              else if (dec.param7 == MAV_MOUNT_MODE_MAVLINK_TARGETING)
                setCameraRotation(dec.param1, dec.param2, dec.param3);
              else if (dec.param7 == MAV_MOUNT_MODE_GPS_POINT) {
                //We need to have GPS coordinates, for this we have to subscribe for MAV_DATA_STREAM_POSITION?
              }
              break;
            }
          }
          break;
        }
      }
    }
  }
}

