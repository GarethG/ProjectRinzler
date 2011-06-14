#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h> 
#include <fcntl.h> 

#include "ros/ros.h"
//#include "sonarDriver.h"

using namespace std;

//INPUT - gradients, binIndex // also later scale in meters and number of samples

//250 BINS
//BINS 0 to 255
//Bearing 0 to 5760 * (1/16) , is 5760 max?

// scale * binIndex / totalSamples
// 20Meters * bin55 / 200 samples

//clockwise, zero degrees behind


int main( int argc, char **argv )
{
char wait; 
float grad = 3000;
float bearing = 0;
int scale = 20;
int samples = 200;
int binIndex = 55;
int range = 0;

//while(1){
	
	range = ((scale * binIndex) / samples);
	bearing = ( grad * 0.0625);

	cout << "Range: " << range << " Meters" << endl;
	cout << "Bearing: " << bearing << " Degrees" << endl;
	
//	cout << "press any character and enter to continue" << endl;
//	cin >> wait;

//}// end while
	return 0; 
}// end main










