/* Defines */

#define	ZEROX	500
#define	ZEROY	500
#define	ZEROZ	500

#define X	0
#define	Y	1
#define	Z	3
#define GO	2

//#define FORCE
#define GOBUTTON

#define VREFH	5000.0
#define VREFL	2500.0

#define VON	4000.0

#define	ADCRES	1023.0

/* Globals */

struct acc_type{
	char invert;
	float zeroG;
	unsigned int rate;
	float R;
}acc[3];

unsigned int accRaw[8];

float goTmp=0.0;

/* Function Declerations */

unsigned int checkGo(void);
void initADC(void);
void readADC(void);
void findForce(void);
