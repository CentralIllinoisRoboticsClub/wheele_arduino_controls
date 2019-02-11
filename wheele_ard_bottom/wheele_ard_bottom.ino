/****************************************************************************
Based on tutorial by Stephen McCoy. 
Original tutorial available here: http://www.instructables.com/id/CAN-Bus-Sniffing-and-Broadcasting-with-Arduino
Used with permission 2016. License CC By SA. 

Distributed as-is; no warranty is given.
*************************************************************************/

#include <mcp_can.h>
#include <SPI.h>
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

#include <PololuMaestro.h>
MicroMaestro maestro(Serial);

//***************CAN IDs**************
#define RC_CMD_CAN_ID 0x101
#define ENC_CAN_ID 0x105
#define BUMP_CAN_ID 0x121
#define TEST_CAN_ID 0x321
#define GYRO_CAN_ID 0x131
#define ACCEL_CAN_ID 0x132
#define COMPASS_CAN_ID 0x133
#define BATT_CAN_ID 0x140

#define MAESTRO_CMD_CAN_ID 0x102 //Bytes 0 to 

//**************Bumper*********
#define BUMP_PIN 8
//**************Encoders*************
#include <PinChangeInt.h>
#define RIGHT_ENC_A 3
#define RIGHT_ENC_B 4
#define LEFT_ENC_A 14 //A0
#define LEFT_ENC_B 15 //A1
//*************Maestro Servos/ESCs***
// Bottom Node expects Top Node to resolve auto vs. manual mode
#define RF_SERVO 0
#define RR_SERVO 1
#define LR_SERVO 2
#define LF_SERVO 3
#define RIGHT_ESC 4
#define LEFT_ESC 5
#define ALPHA_ESC 0.1
#define N_MAESTRO_CMDS 6
#define ZERO_CMD 6000
#define SLOW_CMD_MAG 300
//*************Data Frequency, Periods*********
#define ENC_PERIOD 50 //msec
#define BUMP_PERIOD 100 //msec
//********************************Setup Loop*********************************//

uint16_t timeBump, timeEnc, timeCmd;
long prev_enc_left, prev_enc_right;

int16_t enc_left = 0, enc_right = 0;

unsigned cmd[N_MAESTRO_CMDS] = {ZERO_CMD, ZERO_CMD, ZERO_CMD, ZERO_CMD, ZERO_CMD, ZERO_CMD}; //0 to 5 are servo,ESC cmds

void setup() {
  Serial.begin(9600);
  //Serial.println("CAN Write - Testing transmission of CAN Bus messages");
  delay(200);
  
  while (CAN_OK != CAN.begin(CAN_1000KBPS))              // init can bus : baudrate = 250K
  {
      /*Serial.println("CAN BUS Module Failed to Initialized");
      Serial.println("Retrying....");*/
      delay(500);
      
  }
  
  pinMode(BUMP_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);
  attachInterrupt(1,right_enc_tick, CHANGE);
  attachPinChangeInterrupt(LEFT_ENC_A,left_enc_tick,CHANGE);
  
  delay(200);
  //bno.setExtCrystalUse(true);
  timeBump = millis();
  timeEnc = millis();
  timeCmd = millis();
}

//********************************Main Loop*********************************//

void loop() 
{

  if(CAN_MSGAVAIL == CAN.checkReceive())
  {
    unsigned int rxId;
    unsigned char len = 0;
    unsigned char msg[8];
    // MUST CALL readMsgBuf BEFORE getCanId()
    CAN.readMsgBuf(&len, msg);
    rxId= CAN.getCanId();
    if(rxId == MAESTRO_CMD_CAN_ID)
    {
      timeCmd = millis();
      CAN2Maestro(msg, N_MAESTRO_CMDS, cmd);
      //Serial.print("cmd5: ");
      //Serial.println(cmd[5]);    
    }
  }
  if(timeSince(timeCmd) > 10000)
  {
    maestro.setTarget(LEFT_ESC, ZERO_CMD);
    maestro.setTarget(RIGHT_ESC, ZERO_CMD);
    cmd[LEFT_ESC] = ZERO_CMD;
    cmd[RIGHT_ESC] = ZERO_CMD;
    timeCmd = millis();
  }

  //SEND Encoders
  if(timeSince(timeEnc) > ENC_PERIOD)
  {
    tx_can(ENC_CAN_ID, enc_left, enc_right, 0, 0);
    //Serial.println(enc_left);
    timeEnc = millis();
  }
  
  //SEND Bumper
  if(timeSince(timeBump) > BUMP_PERIOD)
  {
    int16_t bumper = digitalRead(BUMP_PIN);
    tx_can(BUMP_CAN_ID, bumper, 0, 0, 0);
    timeBump = millis();
  }
  
  /*if(rx_can(&msg))
  {
    if(msg.id == 0x202)
    {
      Serial.print("rx msg: ");
      Serial.print(convertCAN(msg,0));
      Serial.print(", ");
      Serial.println(convertCAN(msg,2));
      Serial.println("enc left, enc right:");
      Serial.print(enc_left); Serial.print(", ");
      Serial.println(enc_right);
    }
  }*/

}

uint8_t tx_can(uint16_t id, int16_t num1, int16_t num2, int16_t num3, int16_t num4)
{
  uint16_t raw1 = num1 + 32768;
  uint16_t raw2 = num2 + 32768;
  uint16_t raw3 = num3 + 32768;
  uint16_t raw4 = num4 + 32768;
  
  /*tCAN message;
  message.id = id; //0x631; //formatted in HEX
  message.header.rtr = 0;
  message.header.length = 8; //formatted in DEC */
  unsigned char data[8];
  data[0] = getByte(raw1,1);
  data[1] = getByte(raw1,0);
  data[2] = getByte(raw2,1);
  data[3] = getByte(raw2,0);
  data[4] = getByte(raw3,1);
  data[5] = getByte(raw3,0);
  data[6] = getByte(raw4,1);
  data[7] = getByte(raw4,0);
  
  CAN.sendMsgBuf(id,0, 8, data); //id, 0 means NOT extended ID
  return 0;
  
}

int16_t convertCAN(unsigned char data[], unsigned int start_byte) //start byte is 0 to 7, left to right
{
  return (data[start_byte] << 8 ) + (data[start_byte+1]) - 32768;
}

int8_t getCANByte(unsigned char data[], unsigned int start_byte, bool isSigned)
{
  int8_t offset = 0;
  if(isSigned)
  {
    offset = -128;
  }
  return (data[start_byte] + offset);
}

void CAN2Maestro(unsigned char data[], unsigned n, unsigned cmd[])
{
  unsigned prev_cmd = 6000;
  for(unsigned k=0; k<n; ++k)
  {
    // +/-100 was the input, unsigned byte will be 28 to 228, output is 4000 to 8000
    prev_cmd = cmd[k];
    cmd[k] =  20*(data[k] + 172); //20*(byte-128 + 300)
    if(4000 <= cmd[k] && cmd[k] <= 8000)
    {
      if(k==LEFT_ESC || k==RIGHT_ESC) //ESC cmds
      {
        /*Serial.print("prev, cmd: "); Serial.print(", ");
        Serial.print(prev_cmd); Serial.print(", ");
        Serial.println(cmd[k]);*/
        cmd[k] = unsigned( float(prev_cmd)*(1.0-ALPHA_ESC) + float(cmd[k])*ALPHA_ESC );
      }
      maestro.setTarget(k, cmd[k]);
    }
    else
    {
      cmd[k] = prev_cmd;
    }
  }
}

uint8_t getByte(uint16_t x, unsigned int n)
{
  return (x >> 8*n) & 0xFF;
}

void left_enc_tick()
{
  // modify using PORT operations for efficiency
  if(digitalRead(LEFT_ENC_A) != digitalRead(LEFT_ENC_B))
  {
    enc_left--;
  }
  else
  {
    enc_left++;
  }
}

void right_enc_tick()
{
  // modify using PORT operations for efficiency
  if(digitalRead(RIGHT_ENC_A) != digitalRead(RIGHT_ENC_B))
  {
    enc_right--;
  }
  else
  {
    enc_right++;
  }
}

uint16_t timeSince(uint16_t startTime)
{
  return (uint16_t)(millis() - startTime);
}
