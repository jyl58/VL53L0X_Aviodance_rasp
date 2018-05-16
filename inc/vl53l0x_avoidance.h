#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h> 
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <iostream>

#include "mavlink_types.h"
#include "ardupilotmega/mavlink.h"
#include "ardupilotmega/ardupilotmega.h"
#include "vl53l0x_avoidance_def.h"
#include "serial.h"

using namespace std;
#define VL530_MAX_COUNT 8
#define VL530_MAX_DISTANCE 120;
#define VL530_MIN_DISTANCE 40;
class CmdLineParser
{
    int argc;
    char **argv;
public:
    CmdLineParser(int _argc, char **_argv)
        : argc(_argc)
        , argv(_argv)
    {
    }
    bool operator[](string param)
    {
        int idx = -1;
        for (int i = 0; i < argc && idx == -1; i++)
            if (string(argv[i]) == param)
                idx = i;
        return (idx != -1);
    }
    string operator()(string param, string defvalue = "-1")
    {
        int idx = -1;
        for (int i = 0; i < argc && idx == -1; i++)
            if (string(argv[i]) == param)
                idx = i;
        if (idx == -1)
            return defvalue;
        else
            return (argv[idx + 1]);
    }
};

class VL53L0X_Avoidance{
public:
	VL53L0X_Avoidance();
	~VL53L0X_Avoidance();
	void init_gpio();
	void open_i2c();
	void open_uart(const string uart_dev,int baudrate=115200);
	void init_vl53l0x();
	void update_range();
	uint16_t get_distance(uint8_t id);
	void send_range_to_fc();
private:
	void send_range_to_fc(uint8_t id);
	bool init_vl53l0x(uint8_t id);
	bool update_range(uint8_t id);
	void print_pal_error(VL53L0X_Error Status);
	VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev);
	
	struct VL53L0X Avoidance_Device[VL530_MAX_COUNT];
	int i2c_fd=-1;
	Serial* _uart;
	uint8_t system_id=0,component_id=0;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
};