/* Defines */

#define BAUDRATE B115200
#define SENDBUFFSIZE 82
//Sonar commands
#define mtNull					0
#define mtVersionData			1	
#define mtHeadData				2
#define mtSpectData				3
#define mtAlive					4
#define mtPrgAck				5
#define mtBBUserData			6
#define mtTestData				7
#define mtAuxData				8
#define mtAdcData				9
#define mtAdcReq				10
#define mtLanStatus				13
#define mtSetTime				14
#define mtTimeout				15
#define mtReBoot				16
#define mtPerformanceData		17
#define mtHeadCommand			19
#define mtEraseSector			20
#define mtProgBlock				21
#define mtCopyBootBlk			22
#define mtSendVersion			23
#define mtSendBBUser			24
#define mtSendData				25
#define mtSendPerformanceData	26
#define mtStopAlive				66

/* Function Declerations */

int open_port(void);
void config_port(void);
int write_port(	unsigned char sendBuffer[SENDBUFFSIZE], 
				unsigned int sendSize);
int read_port(void);

unsigned int getU32(unsigned int tmp1, 
					unsigned int tmp2, 
					unsigned int tmp3, 
					unsigned int tmp4);
unsigned int getU16(unsigned int tmp1, 
					unsigned int tmp2);
unsigned int getU8(void);

int sortPacket(void);
int returnMsg(void);

void makePacket(int command);
void makeHeadPacket(unsigned int range, 
					unsigned int startAngle, 
					unsigned int endAngle, 
					unsigned int ADspan, 
					unsigned int ADlow, 
					unsigned int gain,
					unsigned int ADInterval, 
					unsigned int numBins);

int packetLength(int flag);

int initSonar( void );
int sendBB( void );
int headSetup( void );
int requestData( void );
