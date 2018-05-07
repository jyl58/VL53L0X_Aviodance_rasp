#include "vl53l0x_avoidance.h"
#include <stdlib.h>
#include <sys/time.h>
#define SEND_DATA_FRE 10  //10 hz
//ms 
double tic(){
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec*1000 + ((double)t.tv_usec)*0.001);
}
int main(int argc, char **argv)
{
	CmdLineParser cml(argc, argv);
	int baud_rate=115200;
	string dev;
	if (cml["-b"]) baud_rate =atoi(cml("-b").c_str());
	if (cml["-d"]){
		dev=cml("-d");
	}else{
		cout<<"need uart dev"<<endl;
		exit(1);
	}
	VL53L0X_Avoidance *_vl53l0x_avoidance= new VL53L0X_Avoidance();
	_vl53l0x_avoidance->open_uart(dev,baud_rate);
	_vl53l0x_avoidance->init_gpio();
	_vl53l0x_avoidance->open_i2c();
	_vl53l0x_avoidance->init_vl53l0x();
	double last_send_time=tic();
	while(1){
		_vl53l0x_avoidance->update_range();
		for(int i=0;i<VL530_MAX_COUNT;i++){
			cout<<i<<" :distance: "<<_vl53l0x_avoidance->get_distance(i)<<" mm\n"<<endl;
		}
		//send to fc
		if(tic()-last_send_time>1000/SEND_DATA_FRE){
			_vl53l0x_avoidance->send_range_to_fc();
			last_send_time=tic();
		}
	}
	delete _vl53l0x_avoidance;
}