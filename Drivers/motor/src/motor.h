/* Defines */

#define	PWM_FREQUENCY_US	20000
#define	MIN_DUTY_CYCLE_US	1000
#define	MAX_DUTY_CYCLE_US	2000
#define	ZERO_DUTY_CYCLE_US	(MIN_DUTY_CYCLE_US + MAX_DUTY_CYCLE_US)/2

#define	RIGHT_MOTOR_CHANNEL	3
#define	LEFT_MOTOR_CHANNEL	5
#define	FRONT_MOTOR_CHANNEL	4
#define	BACK_MOTOR_CHANNEL	2

#define	LEFT_PWM_OFFSET		0
#define	RIGHT_PWM_OFFSET	-20
#define	FRONT_PWM_OFFSET	0
#define	BACK_PWM_OFFSET		-20

/* Globals */

/* Function Declarations */

void initMotors(void);
