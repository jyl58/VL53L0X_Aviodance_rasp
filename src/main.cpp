#include "vl53l0x_avoidance.h"

int main(int argc, char **argv)
{
	CmdLineParser cml(argc, argv);
	int baud_rate=115200;
	string dev;
	if (cml["-b"]) baud_rate =stoi(cml("-b"));
	if (cml["-d"]){
		dev=cml("-d");
	}else{
		cout<<"need uart dev"<<endl;
		exit(1);
	}
	VL53L0X_Avoidance *_vl53l0x_avoidance=new VL53L0X_Avoidance();
	_vl53l0x_avoidance->open_uart(dev,baud_rate);
	_vl53l0x_avoidance->init_gpio();
	_vl53l0x_avoidance->open_i2c();
	_vl53l0x_avoidance->init_vl53l0x();
	while(1){
		_vl53l0x_avoidance->update_range();
		for(int i=0;i<VL530_MAX_COUNT;i++){
			cout<<i<<" :distance: "<<_vl53l0x_avoidance->get_distance(i)<<" mm\n"<<endl;
		}
	}
	delete _vl53l0x_avoidance;
}