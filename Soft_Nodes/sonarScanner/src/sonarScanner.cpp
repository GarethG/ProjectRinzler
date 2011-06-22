#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "ros/ros.h"
#include "sonarScanner.h"

#define WIDTH	70
#define HEIGHT 	70

	int imageArray[HEIGHT][WIDTH];

/************************************************
 * 
 * The maximum length of the sonar reading needs to be at most half
 * the diameter of the displayed circle.
 * 
 * *********************************************/

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "sonarScanner");
	
	int i;
	
	while(1)
	{
		for( i = 0; i < 360; i ++ )
		{
		
			pixelPlace( i, genRand(30), genRand(4) );
		
		}
		
		printascii();
		sleep(1);		
		
	}
		
	return 0;
	
}

void pixelPlace( unsigned int theta, unsigned int distance, unsigned opaqueVal )
{
	
	int x, y;
	
	if( theta >= 0 && theta <= 90 )
	{
		x = distance * cos(theta);
		y = distance * sin(theta);
		
		x = x + (WIDTH / 2);
		y = y + (HEIGHT / 2);
		
	}
	else if( theta >=91 && theta <= 180 )
	{
		y = distance * cos(theta);
		x = distance * sin(theta);
		
		x = x + (WIDTH / 2);
		y = (y * -1) + (HEIGHT / 2);		
	}
	else if( theta >= 181 && theta <= 270 )
	{
		x = distance * cos(theta);
		y = distance * sin(theta);		
		
		x = (x * -1) + (WIDTH / 2);
		y = (y * -1) + (HEIGHT / 2);
		
	}
	else
	{
		y = distance * cos(theta);
		x = distance * sin(theta);		
		
		x = (x * -1) + (WIDTH / 2);
		y = y + (HEIGHT / 2);
	}
	
	imageArray[x][y] = 1;
	
}

void printascii( void )
{
	
	int i, j;
		
	for( i = 0; i < HEIGHT; i ++ )
	{
		for( j = 0; j < WIDTH; j++ )
		{
			if(imageArray[i][j] == 1)
			{
				printf("X ");
				imageArray[i][j] = 0;
			}
			else
				printf("- ");
		}
		printf("\n");
	}
}

int genRand( int n )
{
    
    return rand() % n;
    
}
