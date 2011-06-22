#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "ros/ros.h"
#include "sonarScanner.h"

#define WIDTH	41
#define HEIGHT 	41

	int imageArray[HEIGHT][WIDTH];

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "sonarScanner");
	
	int i, j;
	
	for( i = 0; i < 360; i ++ )
	{
		
		pixelPlace( i, 20, 1 );
		
	}
	
	for( i = 0; i < HEIGHT; i ++ )
	{
		for( j = 0; j < WIDTH; j++ )
		{
			if(imageArray[i][j] == 1)
				printf("X ");
			else
				printf("- ");
		}
		printf("\n");
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

int genRand( int n )
{
    
    return rand() % n;
    
}
