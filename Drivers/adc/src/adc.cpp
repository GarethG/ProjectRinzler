#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "roboard.h"
#include "ad79x8.h"


#include "adc.h"


int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	ros::init(argc, argv, "adc");	//inits the driver

	/* Messages and services */

	ros::NodeHandle adcN;

	/* Publish */

	/* Subscribe */

	initADC();

	ROS_INFO("ADC Online");

	ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	while (ros::ok()){
		//ros::spin();
		readADC();
		findForce();
		loop_rate.sleep();
	}

	printf("Shutting Down\n");

	return 0;
}

void initADC(void){
	acc[X].zeroG = (float)ZEROX;
	acc[X].zeroG /= ADCRES;
	acc[X].zeroG *= VREF;
	acc[Y].zeroG = (float)ZEROY;
	acc[Y].zeroG /= ADCRES;
	acc[Y].zeroG *= VREF;
	acc[Z].zeroG = (float)ZEROZ;
	acc[Z].zeroG /= ADCRES;
	acc[Z].zeroG *= VREF;
}

void readADC(void){

	unsigned int val,i;

	if(spi_Init(SPICLK_21400KHZ)){
		for(i=0;i<8;i++){
			accRaw[i] = adc_ReadChannel(i, ADCMODE_RANGE_2VREF,ADCMODE_UNSIGNEDCODING);
			printf("Val at channel %u: is %u\n",accRaw[i],val);
		}
		spi_Close();
	}
	return;
}

void findForce(void){

	unsigned int i;

	for(i=X;i<=Z;i++){
		acc[i].rate = accRaw[i];
	}
	
	for(i=X;i<=Z;i++){
		acc[i].R = (float)acc[i].rate;
		acc[i].R /= ADCRES;
		acc[i].R *= VREF;
		acc[i].R -= acc[i].zeroG
		printf("I read %.3f of force at %d\n",acc[i].R,i);
	}

	return;
}
