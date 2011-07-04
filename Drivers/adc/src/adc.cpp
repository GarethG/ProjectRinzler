#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"

#include "roboard.h"
#include "ad79x8.h"


#include "adc.h"


int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	ros::init(argc, argv, "adc");	//inits the driver

	/* Messages and services */

	ros::NodeHandle adcN;

	/* Publish */
	
	#ifdef FORCE
	ros::Publisher adcXMsg = adcN.advertise<std_msgs::Float32>("adcX", 100);
	ros::Publisher adcYMsg = adcN.advertise<std_msgs::Float32>("adcY", 100);
	ros::Publisher adcZMsg = adcN.advertise<std_msgs::Float32>("adcZ", 100);

	std_msgs::Float32 adcX;
	std_msgs::Float32 adcY;
	std_msgs::Float32 adcZ;
	#endif

	#ifdef GOBUTTON
	ros::Publisher adcGoMsg = adcN.advertise<std_msgs::UInt32>("adcGo", 100);
	std_msgs::UInt32 adcGo;
	#endif

	initADC();

	ROS_INFO("ADC Online");

	ros::Rate loop_rate(10); //how many times a second (i.e. Hz) the code should run

	while (ros::ok()){
		//ros::spin();
		readADC();
		#ifdef FORCE
		findForce();

		adcX.data = acc[X].R;
		adcY.data = acc[Y].R;
		adcZ.data = acc[Z].R;

		adcXMsg.publish(adcX);
		adcYMsg.publish(adcY);
		adcZMsg.publish(adcZ);

		#endif

		#ifdef GOBUTTON

		adcGo.data = checkGo();
		
		adcGoMsg.publish(adcGo);

		#endif

		loop_rate.sleep();
	}

	printf("Shutting Down\n");

	return 0;
}

int checkGo(void){
	goTmp = (float)accRaw[GO];
	goTmp /= ADCRES;
	goTmp *= VREFH;
	
	if(goTmp >= VON){
		return 1; //if we have an appropriate voltage return go
	}
	
	return 0;	//else return stop
}	

void initADC(void){
	#ifdef FORCE
	acc[X].zeroG = (float)ZEROX;
	acc[X].zeroG /= ADCRES;
	acc[X].zeroG *= VREFH;
	acc[Y].zeroG = (float)ZEROY;
	acc[Y].zeroG /= ADCRES;
	acc[Y].zeroG *= VREFH;
	acc[Z].zeroG = (float)ZEROZ;
	acc[Z].zeroG /= ADCRES;
	acc[Z].zeroG *= VREFH;
	#endif
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
		acc[i].R *= VREFH;
		acc[i].R -= acc[i].zeroG;
		printf("I read %.3f of force at %d\n",acc[i].R,i);
	}

	return;
}
