#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <AP_Param.h>
#include <telemetry.h>

// Global parameter class.
//
class Parameters {
   public:
   //////////////////////////////////////////////////////////////////
   // STOP!!! DO NOT CHANGE THIS VALUE UNTIL YOU FULLY UNDERSTAND THE
   // COMMENTS ABOVE. IF UNSURE, ASK ANOTHER DEVELOPER!!!
   static const uint16_t k_format_version = 1;
   //////////////////////////////////////////////////////////////////

   static const uint16_t k_software_type = 10; //0-9 reserved for ArduPilot

   enum {
      // Layout version number, always key zero.
     //
     k_param_format_version = 0,
     k_param_software_type,

     //PID settings
     //
     k_param_pid_pitch,
     k_param_pid_roll,
     k_param_pid_yaw,

     //Input settings
     k_param_input_pitch,
     k_param_input_roll,
     k_param_input_yaw,

     //Mixed input settings
     k_param_mixed_input_pitch,
     k_param_mixed_input_roll,
     k_param_mixed_input_yaw,

     //Motor settings
     k_param_motor_pitch,
     k_param_motor_roll,
     k_param_motor_yaw,

     //Sensor settings
     k_param_sensor_settings_x,
     k_param_sensor_settings_y,
     k_param_sensor_settings_z,


     //Sensor offsets
     k_param_sensor_accelBias_x,
     k_param_sensor_accelBias_y,
     k_param_sensor_accelBias_z,

     k_param_sensor_gyroBias_x,
     k_param_sensor_gyroBias_y,
     k_param_sensor_gyroBias_z,

     //Complementary filter settings.
     k_param_cf_2kp,
     k_param_cf_2ki

   };

   AP_Int16 format_version;
   AP_Int8 software_type;

   AP_Int8 sensor_settings_x;
   AP_Int8 sensor_settings_y;
   AP_Int8 sensor_settings_z;

   AP_Float sensor_accelBias_x;
   AP_Float sensor_accelBias_y;
   AP_Float sensor_accelBias_z;

   AP_Float sensor_gyroBias_x;
   AP_Float sensor_gyroBias_y;
   AP_Float sensor_gyroBias_z;

   AP_Int16 cf_2kp;
   AP_Int16 cf_2ki;

};
void load_parameters(void);

extern const AP_Param::Info var_info[];

#endif /* PARAMETERS_H_ */
