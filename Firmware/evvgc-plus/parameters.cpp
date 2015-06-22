#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v, {def_value : def} }
#define ASCALAR(v, name, def) { aparm.v.vtype, name, Parameters::k_param_ ## v, &aparm.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v, {group_info : class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, &v, {group_info : class::var_info} }

#include "ch.h"
#include "hal.h" //TODO: remove
#include "parameters.h"
#include "attitude.h"
#include "pwmio.h"
#include "mpu6050.h"
#include "eeprom.h"

extern Parameters g;

const AP_Param::Info var_info[] PROGMEM = {

   // @Param: FORMAT_VERSION
   // @DisplayName: Eeprom format version number
   // @Description: This value is incremented when changes are made to the eeprom format
   // @User: Advanced
   GSCALAR(format_version,         "FORMAT_VERSION", 0),

   // @Param: SYSID_SW_TYPE
   // @DisplayName: Software Type
   // @Description: This is used by the ground station to recognise the software type (eg ArduPlane vs ArduCopter)
   // @User: Advanced
   GSCALAR(software_type,          "SYSID_SW_TYPE", Parameters::k_software_type),

   // PID_setting
   //-----------
   // @Group: PITCH_
   // @Path: attitude.cpp
   GOBJECTN(g_pidSettings[0], pid_pitch,"PITCH_", PIDSettings),

   // PID_setting
   //-----------
   // @Group: ROLL_
   // @Path: attitude.cpp
   GOBJECTN(g_pidSettings[1], pid_roll,"ROLL_", PIDSettings),

   // PID_setting
   //-----------
   // @Group: YAW_
   // @Path: attitude.cpp
   GOBJECTN(g_pidSettings[2], pid_yaw, "YAW_", PIDSettings),

   // Input settings
   //-----------
   // @Group: PITCH_
   // @Path: attitude.cpp
   GOBJECTN(g_modeSettings[0], input_pitch, "PITCH_", InputMode),

   // Input settings
   //-----------
   // @Group: ROLL_
   // @Path: attitude.cpp
   GOBJECTN(g_modeSettings[1], input_roll, "ROLL_", InputMode),

   // Input settings
   //-----------
   // @Group: YAW_
   // @Path: attitude.cpp
   GOBJECTN(g_modeSettings[2], input_yaw, "YAW_", InputMode),

   // Mixed input settings
   //-----------
   // @Group: PITCH_
   // @Path: pwmio.cpp
   GOBJECTN(g_mixedInput[0], mixed_input_pitch, "PITCH_", MixedInput),

   // Mixed input settings
   //-----------
   // @Group: ROLL_
   // @Path: pwmio.cpp
   GOBJECTN(g_mixedInput[1], mixed_input_roll, "ROLL_", MixedInput),

   // Mixed input settings
   //-----------
   // @Group: YAW_
   // @Path: pwmio.cpp
   GOBJECTN(g_mixedInput[2], mixed_input_yaw, "YAW_", MixedInput),

   // Motor settings
   //-----------
   // @Group: PITCH_
   // @Path: pwmio.cpp
   GOBJECTN(g_pwmOutput[0], motor_pitch, "PITCH_", PWMOutput),

   // Motor settings
   //-----------
   // @Group: ROLL_
   // @Path: pwmio.cpp
   GOBJECTN(g_pwmOutput[1], motor_roll, "ROLL_", PWMOutput),

   // Motor settings
   //-----------
   // @Group: YAW_
   // @Path: pwmio.cpp
   GOBJECTN(g_pwmOutput[2], motor_yaw, "YAW_", PWMOutput),

   // @Param: SENSOR_SETT_X
   // @DisplayName: Sensor settings bitmask
   // @Description: Bitmap of sensor settings
   // @User: Advanced
   GSCALAR(sensor_settings_x, "SENSOR_SETT_X", 0x00),

   // @Param: SENSOR_SETT_Y
   // @DisplayName: Sensor settings bitmask
   // @Description: Bitmap of sensor settings
   // @User: Advanced
   GSCALAR(sensor_settings_y, "SENSOR_SETT_Y", 0x11),

   // @Param: SENSOR_SETT_Z
   // @DisplayName: Sensor settings bitmask
   // @Description: Bitmap of sensor settings
   // @User: Advanced
   GSCALAR(sensor_settings_z, "SENSOR_SETT_Z", 0x22 | IMU1_AXIS_DIR_POS | IMU2_AXIS_DIR_POS),

   // @Param: ACCELOFF_X
   // @DisplayName: Sensor settings bitmask
   // @Description: Bitmap of sensor settings
   // @User: Advanced
   GSCALAR(sensor_accelBias_x, "ACCELOFF_X", 0),

   // @Param: ACCELOFF_Y
   // @DisplayName: Sensor settings bitmask
   // @Description: Bitmap of sensor settings
   // @User: Advanced
   GSCALAR(sensor_accelBias_y, "ACCELOFF_Y", 0),

   // @Param: ACCELOFF_Z
   // @DisplayName: Sensor settings bitmask
   // @Description: Bitmap of sensor settings
   // @User: Advanced
   GSCALAR(sensor_accelBias_z, "ACCELOFF_Z", 0),

   // @Param: GYROFF_X
   // @DisplayName: Gyro offset
   // @Description: Gyro offset
   // @User: Advanced
   GSCALAR(sensor_gyroBias_x, "GYROFF_X", 0),

   // @Param: GYROFF_Y
   // @DisplayName: Gyro offset
   // @Description: Gyro offset
   // @User: Advanced
   GSCALAR(sensor_gyroBias_y, "GYROFF_Y", 0),

   // @Param: GYROFF_Z
   // @DisplayName: Gyro offset
   // @Description: Gyro offset
   // @User: Advanced
   GSCALAR(sensor_gyroBias_z, "GYROFF_Z", 0),

   // @Param: COMP_F_2KP
   // @DisplayName: Complementary filter setting
   // @Description: Complementary filter setting
   // @User: Advanced
   GSCALAR(cf_2kp, "COMP_F_2KP", 300),

   // @Param: COMP_F_2KI
   // @DisplayName: Complementary filter setting
   // @Description: Complementary filter setting
   // @User: Advanced
   GSCALAR(cf_2ki, "COMP_F_2KI", 100),

   AP_VAREND
};

void load_parameters(void)
{
   //eepromWriteBlock(NULL, NULL, NULL);
   if(!AP_Param::check_var_info()) {
      debugLog("P: EEPROM corruped");
   }

   if(!g.format_version.load() || g.format_version != Parameters::k_format_version) {
      debugLog("P: Wiping EEPROM");
      AP_Param::erase_all();
      uint8_t dump[EEPROM_24C02_SIZE];
      i2cAcquireBus(&I2CD2);
      i2cMasterTransmitTimeout(&I2CD2, EEPROM_24C02_ADDR, 0x00, 1, (uint8_t *)dump, EEPROM_24C02_SIZE, MS2ST(20));
      i2cReleaseBus(&I2CD2);
      g.format_version.set_and_save(Parameters::k_format_version);
   } else {
      AP_Param::load_all();
   }

   //TODO: Better way for sensor settings...
   g_sensorSettings[0] = g.sensor_settings_x;
   g_sensorSettings[1] = g.sensor_settings_y;
   g_sensorSettings[2] = g.sensor_settings_z;

   accelBiasUpdate();
   gyroBiasUpdate();


}
