#pragma once

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#define MAV_SENSOR_ROTATION_NONE    0 /* Roll: 0, Pitch: 0, Yaw: 0 | */
#define MAV_SENSOR_ROTATION_YAW_45  1 /* Roll: 0, Pitch: 0, Yaw: 45 | */
#define MAV_SENSOR_ROTATION_YAW_90  2 /* Roll: 0, Pitch: 0, Yaw: 90 | */
#define MAV_SENSOR_ROTATION_YAW_135 3 /* Roll: 0, Pitch: 0, Yaw: 135 | */
#define MAV_SENSOR_ROTATION_YAW_180 4 /* Roll: 0, Pitch: 0, Yaw: 180 | */
#define MAV_SENSOR_ROTATION_YAW_225 5 /* Roll: 0, Pitch: 0, Yaw: 225 | */
#define MAV_SENSOR_ROTATION_YAW_270 6 /* Roll: 0, Pitch: 0, Yaw: 270 | */
#define MAV_SENSOR_ROTATION_YAW_315 7 /* Roll: 0, Pitch: 0, Yaw: 315 | */

struct ADDR_STRUCT{
	const uint8_t control_gpio_id;
	const uint8_t new_address;
	const uint8_t orientation;
	const char *description;
};
const  struct ADDR_STRUCT vl53l0x_addr_table[8]={
	/*control gpio id ,add,description*/
	{4,0x30,MAV_SENSOR_ROTATION_NONE,"front"},
	{17,0x32,MAV_SENSOR_ROTATION_YAW_45,"right_front"},
	{27,0x34,MAV_SENSOR_ROTATION_YAW_90,"right"},
	{22,0x36,MAV_SENSOR_ROTATION_YAW_135,"right_back"},
	{5,0x38,MAV_SENSOR_ROTATION_YAW_180,"back"},
	{6,0x40,MAV_SENSOR_ROTATION_YAW_225,"left_back"},
	{13,0x42,MAV_SENSOR_ROTATION_YAW_270,"left"},
	{19,0x44,MAV_SENSOR_ROTATION_YAW_315,"left_front"}
};
struct VL53L0X{
	VL53L0X_Dev_t Device;
	uint8_t control_gpio_id;
	uint8_t new_address;
	uint8_t orientation;
	VL53L0X_RangingMeasurementData_t RangingMeasurementData;
};