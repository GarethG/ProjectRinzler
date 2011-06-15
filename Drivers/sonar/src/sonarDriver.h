/* Function Declerations */

int open_port(void);
void config_port(void);
int write_port(void);
int read_port(void);
unsigned int getU32(unsigned int tmp1, unsigned int tmp2, unsigned int tmp3, unsigned int tmp4);
unsigned int getU16(unsigned int tmp1, unsigned int tmp2);
unsigned int getU8(void);
int sortPacket(void);
int returnMsg();
void makePacket(int command);
int clearPacket(void);
void prinfPacket(void);
int packetLength(int flag);
void makeHeadPacket();

/* Defines */

#define BAUDRATE B115200
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


/* **************************************************************
  
 Send Commands

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
#define mtSendBBuser			24
#define mtSendData				{0x40, 0x30, 0x30, 0x30, 0x43, 0x0C, 0x00, 0xFF, 0x02, 0x07, 0x19, 0x80, 0x02, 0xCA, 0x64, 0xB0, 0x03, 0x0A}
#define mtSendPerformanceData	26 
  
								{0x40, 0x30, 0x30, 0x30, 0x38, 0x08, 0x00, 0xFF, 0x02, 0x03, 0x17, 0x80, 0x02, 0x0A }
  
  
 *  ************************************************************** */
