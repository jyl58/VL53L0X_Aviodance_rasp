#include "vl53l0x_avoidance.h"

VL53L0X_Avoidance::VL53L0X_Avoidance(){
	//set addr and control gpio
	for(int i=0;i<VL530_MAX_COUNT;i++){
		Avoidance_Device[i].Device.I2cDevAddr=0x29;	
		Avoidance_Device[i].control_gpio_id=vl53l0x_addr_table[i].control_gpio_id;
		Avoidance_Device[i].new_address=vl53l0x_addr_table[i].new_address;
		Avoidance_Device[i].orientation=vl53l0x_addr_table[i].orientation;
	}
}
void VL53L0X_Avoidance::init_gpio(){
	//init gpio status
	if(wiringPiSetup()<0||wiringPiSetupGpio()<0){
		cout<<"Failed to setup Pi GPIO"<<endl;
		exit(1);
	} 
	for(int i=0;i<VL530_MAX_COUNT;i++){
		pinMode(Avoidance_Device[i].control_gpio_id, OUTPUT);
		digitalWrite(Avoidance_Device[i].control_gpio_id, LOW);
	}
	cout<<"GPIO init done"<<endl;
}
VL53L0X_Avoidance::~VL53L0X_Avoidance(){
	VL53L0X_i2c_close();
	if(_uart!=NULL)
		delete _uart;
}
void  VL53L0X_Avoidance::open_uart(const string uart_dev,int baudrate){
	_uart=new Serial();
	_uart->open(uart_dev,baudrate);
}
void VL53L0X_Avoidance::open_i2c(){
    if ((i2c_fd = open("/dev/i2c-1", O_RDWR)) < 0) {
        /* ERROR HANDLING: you can check errno to see what went wrong */
        cout<<"Failed to open the i2c bus"<<endl;
        exit(1);
    }
}
void VL53L0X_Avoidance::init_vl53l0x(){
	for(int i=0;i<VL530_MAX_COUNT;i++){
		if(!init_vl53l0x(i)){	
			exit(1);
		}
	}
	cout<<"all VL53L0X init done"<<endl;
}
bool VL53L0X_Avoidance::init_vl53l0x(uint8_t id){
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	Status = VL53L0X_ERROR_NONE;
	printf ("Do %s of VL53L0X Init\n",vl53l0x_addr_table[id].description);
	digitalWrite(Avoidance_Device[id].control_gpio_id,  HIGH);
	delay(5);
	//get i2c bus
	Avoidance_Device[id].Device.fd=i2c_fd;
	if (ioctl(i2c_fd, I2C_SLAVE,Avoidance_Device[id].Device.I2cDevAddr) < 0){
		cout<<"Failed to acquire IIc bus access and/or talk to slave."<<endl;
		return false;
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
        Status = VL53L0X_DataInit(&Avoidance_Device[id].Device); // Data initialization
        print_pal_error(Status);
    }else{
		printf ("Call %s of VL53L0X Data Init err!\n",vl53l0x_addr_table[id].description);
		return false;
	}
	if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_StaticInit(&Avoidance_Device[id].Device); // Device Initialization
        // StaticInit will set interrupt by default
        print_pal_error(Status);
    }else{
		printf ("Call of %s VL53L0X_StaticInit err\n", vl53l0x_addr_table[id].description);
		return false;
	}
    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_PerformRefCalibration(&Avoidance_Device[id].Device,&VhvSettings, &PhaseCal); // Device Initialization
        print_pal_error(Status);
    }else{
		printf ("Call of %s VL53L0X_PerformRefCalibration err \n",vl53l0x_addr_table[id].description);
		return false;
	}

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_PerformRefSpadManagement(&Avoidance_Device[id].Device,&refSpadCount, &isApertureSpads); // Device Initialization
        print_pal_error(Status);
    }else{
		printf ("Call of %s VL53L0X_PerformRefSpadManagement err\n",vl53l0x_addr_table[id].description);
		return false;
	}

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_SetDeviceMode(&Avoidance_Device[id].Device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
        print_pal_error(Status);
    }else{
		printf ("Call of %s VL53L0X_SetDeviceMode err\n",vl53l0x_addr_table[id].description);
		return false;
	}
    
    if(Status == VL53L0X_ERROR_NONE)
    {
		Status = VL53L0X_StartMeasurement(&Avoidance_Device[id].Device);
		print_pal_error(Status);
    }else{
		printf ("Call of %s VL53L0X_StartMeasurement err\n",vl53l0x_addr_table[id].description);
		return false;
	}
	return true;
}
void VL53L0X_Avoidance::print_pal_error(VL53L0X_Error Status){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    printf("API Status: %i : %s\n", Status, buf);
}
void VL53L0X_Avoidance::send_range_to_fc(){
	for(int i=0;i<VL530_MAX_COUNT;i++){
		send_range_to_fc(i);
	}
}
void VL53L0X_Avoidance::send_range_to_fc(uint8_t id){
	mavlink_distance_sensor_t distance_mavlink_msg;
	mavlink_message_t mavlink_msg;
	memset(&distance_mavlink_msg, 0, sizeof(mavlink_distance_sensor_t));
    memset(&mavlink_msg, 0, sizeof(mavlink_message_t));
	distance_mavlink_msg.current_distance=get_distance(id)/10;  //mm ->cm
	distance_mavlink_msg.orientation=Avoidance_Device[id].orientation;
	distance_mavlink_msg.max_distance=VL530_MAX_DISTANCE;
	distance_mavlink_msg.min_distance=VL530_MIN_DISTANCE;
	mavlink_msg_distance_sensor_encode(system_id, component_id, &mavlink_msg, &distance_mavlink_msg);
	if(_uart->print((char *)&mavlink_msg, sizeof(mavlink_message_t)) != sizeof(mavlink_message_t))
		cerr << "send mavlink message err" << endl;
}
uint16_t VL53L0X_Avoidance::get_distance(uint8_t id){
	return Avoidance_Device[id].RangingMeasurementData.RangeStatus ==0 ? Avoidance_Device[id].RangingMeasurementData.RangeMilliMeter : 0; //mm
}
void VL53L0X_Avoidance::update_range(){
	for(int i=0;i<VL530_MAX_COUNT;i++){
		if(!update_range(i)){		
		}
	}
}
bool VL53L0X_Avoidance::update_range(uint8_t id){
	Status = VL53L0X_ERROR_NONE;
	//get i2c bus
	if (ioctl(i2c_fd, I2C_SLAVE,Avoidance_Device[id].Device.I2cDevAddr) < 0){
		cout<<"Failed to acquire IIc bus access and/or talk to slave"<<endl;
		return false;
	}
	Status = WaitMeasurementDataReady(&Avoidance_Device[id].Device);
    if(Status == VL53L0X_ERROR_NONE){
		Status = VL53L0X_GetRangingMeasurementData(&Avoidance_Device[id].Device, &Avoidance_Device[id].RangingMeasurementData);
		// Clear the interrupt
		VL53L0X_ClearInterruptMask(&Avoidance_Device[id].Device, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
    } else {
        cout<<"Get of "<<vl53l0x_addr_table[id].description<<"VL53L0X_Measurement err \n"<<endl;
		return false;
    }
	return true;
}
VL53L0X_Error VL53L0X_Avoidance::WaitMeasurementDataReady(VL53L0X_DEV Dev) {
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t NewDatReady=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
            if ((NewDatReady == 0x01) || status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            status = VL53L0X_ERROR_TIME_OUT;
        }
    }
    return status;
}
