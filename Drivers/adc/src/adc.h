/* Defines */

#define	ZEROX	500
#define	ZEROY	500
#define	ZEROZ	500

#define X	1
#define	Y	2
#define	Z	3

#define VREFH	5000.0
#define VREFL	2500.0
#define	ADCRES	1023.0

/* Globals */

struct acc_type{
	char invert;
	float zeroG;
	unsigned int rate;
	float R;
}acc[3];

unsigned int accRaw[8];

/* Function Declerations */

void initADC(void);
void readADC(void);
void findForce(void);
