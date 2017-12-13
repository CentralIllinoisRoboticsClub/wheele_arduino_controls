/****************************************************************************
CAN Write Demo for the SparkFun CAN Bus Shield. 

Written by Stephen McCoy. 
Original tutorial available here: http://www.instructables.com/id/CAN-Bus-Sniffing-and-Broadcasting-with-Arduino
Used with permission 2016. License CC By SA. 

Distributed as-is; no warranty is given.
*************************************************************************/
#define CANSPEED_125 	7		// CAN speed at 125 kbps
#define CANSPEED_250  	3		// CAN speed at 250 kbps
#define CANSPEED_500	1		// CAN speed at 500 kbps
//#include <Canbus.h> // only used CANSPEED_xxx above
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>

//***************CAN IDs**************
#define RC_CMD_CAN_ID 0x101
#define ENC_CAN_ID 0x105
#define BUMP_CAN_ID 0x121
#define TEST_CAN_ID 0x321

//**************Bumper*********
#define BUMP_PIN 8
//**************RC signals IN*********
#define SPEED_PIN 6
#define STEER_PIN 7
//**************Encoders*************
#include <PinChangeInt.h>
#define RIGHT_ENC_A 3
#define RIGHT_ENC_B 4
#define LEFT_ENC_A 14 //A0
#define LEFT_ENC_B 15 //A1
//********************************Setup Loop*********************************//

long timeRC, timeCANPulse;
long prev_enc_left, prev_enc_right;
int prev_steer, prev_speed;

int16_t enc_left = 0, enc_right = 0;

tCAN msg;

void setup() {
  Serial.begin(9600);
  Serial.println("CAN Write - Testing transmission of CAN Bus messages");
  delay(200);
  
  if(mcp2515_init(CANSPEED_500))  //Initialise MCP2515 CAN controller at the specified speed
    Serial.println("CAN Init ok");
  else
    Serial.println("Can't init CAN");
  
  pinMode(BUMP_PIN, INPUT);
  pinMode(SPEED_PIN, INPUT);
  pinMode(STEER_PIN, INPUT);
  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);
  attachInterrupt(1,right_enc_tick, CHANGE);
  attachPinChangeInterrupt(LEFT_ENC_A,left_enc_tick,CHANGE);
    
  delay(200);
  timeRC = millis();
}

//********************************Main Loop*********************************//

void loop() 
{

  int16_t steer_pwm = pulseIn(STEER_PIN, HIGH, 15000);
  int16_t speed_pwm = pulseIn(SPEED_PIN, HIGH, 15000);
  int16_t bumper = digitalRead(BUMP_PIN);
  //SEND RAW RC CMDS
  if(millis() - timeRC > 100)
  {
    tx_can(RC_CMD_CAN_ID, speed_pwm, steer_pwm);
    tx_can(ENC_CAN_ID, enc_left, enc_right);
    tx_can(BUMP_CAN_ID, bumper, 0);
    tx_can(TEST_CAN_ID, 1350, 1500);
    timeRC = millis();
  }
  
  if(rx_can(&msg))
  {
    if(msg.id == 0x202)
    {
      Serial.print("rx msg: ");
      Serial.print(convertCAN(msg,0));
      Serial.print(", ");
      Serial.println(convertCAN(msg,2));
      Serial.println("enc left, enc right, speed pwm, steeer pwm:");
      Serial.print(enc_left); Serial.print(", ");
      Serial.print(enc_right); Serial.print(", ");
      Serial.print(speed_pwm); Serial.print(", ");
      Serial.println(steer_pwm);
    }
  }

}

uint8_t tx_can(uint16_t id, int16_t num1, int16_t num2)
{
  uint16_t raw1 = num1 + 32767;
  uint16_t raw2 = num2 + 32767;
  
  tCAN message;

  message.id = id; //0x631; //formatted in HEX
  message.header.rtr = 0;
  message.header.length = 8; //formatted in DEC
  message.data[0] = getByte(raw1,1);
  message.data[1] = getByte(raw1,0);
  message.data[2] = getByte(raw2,1);
  message.data[3] = getByte(raw2,0);
  message.data[4] = 0x00;
  message.data[5] = 0x00;
  message.data[6] = 0x00;
  message.data[7] = 0x00;
  
  mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
  return mcp2515_send_message(&message);
}
/*
typedef struct
{
	uint16_t id;
	struct {
		int8_t rtr : 1;
		uint8_t length : 4;
	} header;
	uint8_t data[8];
} tCAN;
*/
boolean rx_can(tCAN *message)
{
  if (mcp2515_check_message()) 
  {
    if (mcp2515_get_message(message)) 
    {
      //if(message.id == 0x620 and message.data[2] == 0xFF)  //uncomment when you want to filter
         
       Serial.print("ID: ");
       Serial.print(message->id,HEX);
       Serial.print(", ");
       Serial.print("Data: ");
       Serial.print(message->header.length,DEC);
       for(int i=0;i<message->header.length;i++) 
       {	
          Serial.print(message->data[i],HEX);
          Serial.print(" ");
       }
       Serial.println("");
     }
     else
     {
       return false;
     }
  }
  else
  {
    return false;
  }
  return true;
}

uint16_t convertCAN(tCAN message, unsigned int start_byte) //start byte is 0 to 7, left to right
{
  return (message.data[start_byte] << 8 ) + (message.data[start_byte+1]) - 32767;
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