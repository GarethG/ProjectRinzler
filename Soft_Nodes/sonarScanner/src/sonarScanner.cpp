#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <iostream>
#include <GL/glfw.h> // Include OpenGL Framework library

#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/ByteMultiArray.h"

#include "sonarScanner.h"

#define WIDTH	600
#define HEIGHT 	600

int imageArray[HEIGHT][WIDTH];
unsigned int imgx = 0, imgy = 0;

/************************************************
 * 
 * The maximum length of the sonar reading needs to be at most half
 * the diameter of the displayed circle.
 * 
 * *********************************************/

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "sonarScanner");
	
	/* Messages and services */

	ros::NodeHandle scannerN;	
	
	int i;
	
	ros::Subscriber sub1 = scannerN.subscribe("sonarBearing", 100, bearingCallback);
	ros::Subscriber sub2 = scannerN.subscribe("sonarBins", 100, binsCallback);
	
	// Frame counter and window settings variables
	int frame      = 0, width  = WIDTH, height      = HEIGHT;
	int redBits    = 8, greenBits = 8,   blueBits    = 8;
	int alphaBits  = 8, depthBits = 0,   stencilBits = 0;
 
	// Flag to keep our main loop running
	bool running = true;
 
	// Initialise glfw
	glfwInit();
 
	// Create a window
	if(!glfwOpenWindow(width, height, redBits, greenBits, blueBits, alphaBits, depthBits, stencilBits, GLFW_WINDOW))
	{
		printf("Failed to open window!\n");
		glfwTerminate();
		return 0;
		}
 
	// Call our initGL function to set up our OpenGL options
	initGL(width, height);
	
	while (running == true)
	{

		// Get ros subscriptions
		ros::spinOnce();
		//Do trig to get the xy position of the pixels for the image
		pixelPlace( (int) (bearing  / 17.775), (int) bins * 6, genRand(255) );

		// Increase our frame counter
		frame++;
		//Debug
		printf("Bearing : %f, Distance : %f, X : %d, Y : %d\n",  bearing / 17.775, bins * 6, imgx, imgy);
		
		// Draw the returned xy pixel on the image
		drawScene(imgx, imgy);
 
		// exit if ESC was pressed or window was closed
		running = !glfwGetKey(GLFW_KEY_ESC) && glfwGetWindowParam(GLFW_OPENED);
	
		//printascii();
		//sleep(1);		
		
	}
	
	glfwTerminate();		
	return 0;
	
}
/****************************************************************
 * Do trig to get the xy coordinates of the thing
 * add on half width and half height so it's centred and some things need
 * flipping to make them right 
 * *********************************************************/
void pixelPlace( unsigned int theta, unsigned int distance, unsigned opaqueVal )
{
	
	int x, y;
	if( theta > 0 && theta < 90 )
	{
		//printf("< 90\n");
		x = distance * cos(theta);
		y = distance * sin(theta);
		printf("%d -- %d\n", x, y);
		imgx = x + (WIDTH / 2);
		imgy = y + (HEIGHT / 2);

		
	}
	else if( theta > 91 && theta < 180 )
	{
		//printf("> 90 && < 180\n");
		y = distance * cos(theta);
		x = distance * sin(theta);
		
		imgx = x + (WIDTH / 2);
		imgy = (y * -1) + (HEIGHT / 2);		

	}
	else if( theta > 181 && theta < 270 )
	{
		//printf("> 181 && < 270\n");
		x = distance * cos(theta);
		y = distance * sin(theta);		
		
		imgx = (x * -1) + (WIDTH / 2);
		imgy = (y * -1) + (HEIGHT / 2);
		
	}
	else
	{
		//printf("> 271 && < 360\n");
		y = distance * cos(theta);
		x = distance * sin(theta);		
		
		imgx = (x * -1) + (WIDTH / 2);
		imgy = y + (HEIGHT / 2);
	}
	
	//imageArray[y][x] = 1;
	
}
/****************************************************************
 * print sonar readings in terminal, mega hacks. no longer used
 * ***********************************************************/
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
/***************************************************************
 * set up an image using opengl
 * *************************************************************/
void initGL(int width, int height)
{
	// ----- Window and Projection Settings -----
 
	// Set the window title
	glfwSetWindowTitle("GLFW Basecode");
 
	// Setup our viewport to be the entire size of the window
	glViewport(0, 0, (GLsizei)width, (GLsizei)height);
 
	// Change to the projection matrix, reset the matrix and set up orthagonal projection (i.e. 2D)
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, width, height, 0, 0, 1); // Paramters: left, right, bottom, top, near, far
 
	// ----- OpenGL settings -----
 
	glfwSwapInterval(1); 		// Lock to vertical sync of monitor (normally 60Hz, so 60fps)
 
	glEnable(GL_SMOOTH);		// Enable (gouraud) shading
 
	glDisable(GL_DEPTH_TEST); 	// Disable depth testing
 
	glEnable(GL_BLEND);		// Enable blending (used for alpha) and blending function to use
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
 
	glLineWidth(5.0f);		// Set a 'chunky' line width
 
	glEnable(GL_LINE_SMOOTH);	// Enable anti-aliasing on lines
 
	glPointSize(5.0f);		// Set a 'chunky' point size
 
	glEnable(GL_POINT_SMOOTH);	// Enable anti-aliasing on points
}
/*************************************************************
 * draw x, y position in red on the image
 * ***********************************************************/
void drawScene(unsigned int x, unsigned int y)
{
	
	int i, j;
	
	// Clear the screen
	//glClear(GL_COLOR_BUFFER_BIT);
 
	// Reset the matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
 
	// ----- Draw stuff! -----
 
	glBegin(GL_POINTS);
	glColor3ub(255, 0, 0);
	
	//for( i = 0; i < HEIGHT; i ++ )
	//{
	//	for( j = 0; j < WIDTH; j++ )
	//	{
	//		if(imageArray[i][j] == 1)
	//		{
				glVertex2f(x, y);
				glVertex2f(0, 0);
	//			imageArray[i][j] = 0;
	//		}

	//	}
	//}
			
	glEnd();
 
	// ----- Stop Drawing Stuff! ------
 
	glfwSwapBuffers(); // Swap the buffers to display the scene (so we don't have to watch it being drawn!)
}

/*************************************************
** Returns the sonar bearing **
*************************************************/

void bearingCallback(const std_msgs::Float32::ConstPtr& sonarBearing){
bearing = sonarBearing->data;
return;
}

/*************************************************
** Returns the sonar bearing **
*************************************************/

void binsCallback(const std_msgs::Float32::ConstPtr& sonarBins){
bins = sonarBins->data;
return;
}
