#include <stdlib.h>
#include <stdio.h>
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include <linux/i2c-dev.h>
#include <sys/time.h>
#include <wiringPi.h> 
#include <sys/ioctl.h>
#include <fcntl.h>

#define VERSION_REQUIRED_MAJOR 1
#define VERSION_REQUIRED_MINOR 0
#define VERSION_REQUIRED_BUILD 2

#define VL530_MAX_COUNT 2

struct ADDR_STRUCT{
	const uint8_t control_gpio_id;
	const uint8_t new_address;
	const char *description;
};
const  struct ADDR_STRUCT vl53l0x_addr_table[8]={
	/*control gpio id ,add,description*/
	{4,0x30,"front"},
	{17,0x32,"right_front"},
	{27,0x34,"right"},
	{22,0x36,"right_back"},
	{5,0x38,"back"},
	{6,0x40,"left_back"},
	{13,0x42,"left"},
	{19,0x44,"left_front"}
};
struct VL53L0X{
	VL53L0X_Dev_t Device;
	uint8_t control_gpio_id;
	uint8_t new_address;
};

struct VL53L0X Avoidance_Device[VL530_MAX_COUNT];
VL53L0X_RangingMeasurementData_t    RangingMeasurementData[VL530_MAX_COUNT];
int i2c_fd=-1;
VL53L0X_Error Status = VL53L0X_ERROR_NONE;

void print_pal_error(VL53L0X_Error Status){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    printf("API Status: %i : %s\n", Status, buf);
}

void print_range_status(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    uint8_t RangeStatus;

    /*
     * New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
     */

    RangeStatus = pRangingMeasurementData->RangeStatus;

    VL53L0X_GetRangeStatusString(RangeStatus, buf);
    printf("Range Status: %i : %s\n", RangeStatus, buf);

}

VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t NewDatReady=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
            if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return Status;
}

VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t StopCompleted=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
            if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return Status;
}

 void init_gpio(){
	//set addr and control gpio
	int i=0;
	for(i=0;i<VL530_MAX_COUNT;i++){
		Avoidance_Device[i].Device.I2cDevAddr=0x29;	
		Avoidance_Device[i].control_gpio_id=vl53l0x_addr_table[i].control_gpio_id;
		Avoidance_Device[i].new_address=vl53l0x_addr_table[i].new_address;
	}
	//init gpio status
	if(wiringPiSetup()<0||wiringPiSetupGpio()<0){
		printf("Failed to setup Pi GPIO");
		exit(1);
	} 
	for(i=0;i<VL530_MAX_COUNT;i++){
		pinMode(Avoidance_Device[i].control_gpio_id, OUTPUT);
		digitalWrite(Avoidance_Device[i].control_gpio_id, LOW);
	}
	printf("GPIO init done");
}
int init_vl53l0x(uint8_t id){
	uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
	Status = VL53L0X_ERROR_NONE;
	
	digitalWrite(Avoidance_Device[id].control_gpio_id,  HIGH);
	delay(5);
	//get i2c bus
	Avoidance_Device[id].Device.fd=i2c_fd;
	if (ioctl(i2c_fd, I2C_SLAVE,Avoidance_Device[id].Device.I2cDevAddr) < 0){
		printf("Failed to acquire IIc bus access and/or talk to slave.");
		return -1;
	}
	if(Status == VL53L0X_ERROR_NONE){
		Status =VL53L0X_SetDeviceAddress(&Avoidance_Device[id].Device,Avoidance_Device[id].new_address);
	}
	if(Status == VL53L0X_ERROR_NONE){
		Avoidance_Device[id].Device.I2cDevAddr=Avoidance_Device[id].new_address/2;
		if (ioctl(i2c_fd, I2C_SLAVE,Avoidance_Device[id].Device.I2cDevAddr) < 0){
			printf("Failed to acquire IIc bus access and/or talk to slave.");
			return -1;
		}
	}
	// End of implementation specific
    if(Status == VL53L0X_ERROR_NONE){
		printf ("Call %s of VL53L0X_DataInit\n",vl53l0x_addr_table[id].description);
        Status = VL53L0X_DataInit(&Avoidance_Device[id].Device); // Data initialization
        print_pal_error(Status);
    }else{
		return -1;
	}
	if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_StaticInit(&Avoidance_Device[id].Device); // Device Initialization
        // StaticInit will set interrupt by default
        print_pal_error(Status);
    }else{
		printf ("Call of %s VL53L0X_StaticInit err\n", vl53l0x_addr_table[id].description);
		return -1;
	}
    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_PerformRefCalibration(&Avoidance_Device[id].Device,&VhvSettings, &PhaseCal); // Device Initialization
        print_pal_error(Status);
    }else{
		printf ("Call of %s VL53L0X_PerformRefCalibration err \n",vl53l0x_addr_table[id].description);
		return -1;
	}

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_PerformRefSpadManagement(&Avoidance_Device[id].Device,&refSpadCount, &isApertureSpads); // Device Initialization
        print_pal_error(Status);
    }else{
		printf ("Call of %s VL53L0X_PerformRefSpadManagement err\n",vl53l0x_addr_table[id].description);
		return -1;
	}

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_SetDeviceMode(&Avoidance_Device[id].Device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
        print_pal_error(Status);
    }else{
		printf ("Call of %s VL53L0X_SetDeviceMode err\n",vl53l0x_addr_table[id].description);
		return -1;
	}
    
    if(Status == VL53L0X_ERROR_NONE)
    {
		Status = VL53L0X_StartMeasurement(&Avoidance_Device[id].Device);
		print_pal_error(Status);
    }else{
		printf ("Call of %s VL53L0X_StartMeasurement err\n",vl53l0x_addr_table[id].description);
		return -1;
	}
	return 1;
}
 void init_all_vl53l0x(){
	 int i=0;
	 for(i=0;i<VL530_MAX_COUNT;i++){
		if(!init_vl53l0x(i)){	
			exit(1);
		}
	}
 }  
    
VL53L0X_Error rangingTest(uint8_t id)
{
    VL53L0X_Dev_t *pMyDevice=&Avoidance_Device[id].Device;
	VL53L0X_RangingMeasurementData_t   *pRangingMeasurementData    = RangingMeasurementData+id;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
	if (ioctl(i2c_fd, I2C_SLAVE,Avoidance_Device[id].Device.I2cDevAddr) < 0){
		printf("Failed to acquire IIc bus access and/or talk to slave.");
	}
    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = WaitMeasurementDataReady(pMyDevice);

        if(Status == VL53L0X_ERROR_NONE)
        {
                Status = VL53L0X_GetRangingMeasurementData(pMyDevice, pRangingMeasurementData);
                printf("id: %d  get %d mm\n",id, pRangingMeasurementData->RangeMilliMeter);
                // Clear the interrupt
                VL53L0X_ClearInterruptMask(pMyDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
        } else {
        }
    }
    return Status;
}
void open_i2c(){
    if ((i2c_fd = open("/dev/i2c-1", O_RDWR)) < 0) {
        /* ERROR HANDLING: you can check errno to see what went wrong */
        printf ("Failed to open the i2c bus\n");
        exit(1);
    }
}
int main(int argc, char **argv)
{
    VL53L0X_Version_t                   Version;
    VL53L0X_Version_t                  *pVersion   = &Version;
    VL53L0X_DeviceInfo_t                DeviceInfo;

    int32_t status_int;

    printf ("VL53L0X PAL Continuous Ranging example\n\n");

    /*
     *  Get the version of the VL53L0X API running in the firmware
     */

    if(Status == VL53L0X_ERROR_NONE)
    {
        status_int = VL53L0X_GetVersion(pVersion);
        if (status_int != 0)
            Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    /*
     *  Verify the version of the VL53L0X API running in the firmrware
     */

    if(Status == VL53L0X_ERROR_NONE)
    {
        if( pVersion->major != VERSION_REQUIRED_MAJOR ||
            pVersion->minor != VERSION_REQUIRED_MINOR ||
            pVersion->build != VERSION_REQUIRED_BUILD )
        {
            printf("VL53L0X API Version Error: Your firmware has %d.%d.%d (revision %d). This example requires %d.%d.%d.\n",
                pVersion->major, pVersion->minor, pVersion->build, pVersion->revision,
                VERSION_REQUIRED_MAJOR, VERSION_REQUIRED_MINOR, VERSION_REQUIRED_BUILD);
        }
    }
	open_i2c();
	init_gpio();
	init_all_vl53l0x();
   
    if(Status == VL53L0X_ERROR_NONE)
    {
		while(1){
			int i=0;
			for(i=0;i<VL530_MAX_COUNT;i++){
				Status = rangingTest(i);
			}
		}
    }

    print_pal_error(Status);

    /*
     *  Disconnect comms - part of VL53L0X_platform.c
     */

    printf ("Close Comms\n");
    VL53L0X_i2c_close();


    print_pal_error(Status);
    
    return (0);
}

