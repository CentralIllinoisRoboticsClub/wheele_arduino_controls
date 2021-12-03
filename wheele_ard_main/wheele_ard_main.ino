/****************************************************************************
Based on tutorial by Stephen McCoy. 
Original tutorial available here: http://www.instructables.com/id/CAN-Bus-Sniffing-and-Broadcasting-with-Arduino
Used with permission 2016. License CC By SA. 

Distributed as-is; no warranty is given.
*************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define CANSPEED_125 	7		// CAN speed at 125 kbps
#define CANSPEED_250  	3		// CAN speed at 250 kbps
#define CANSPEED_500	1		// CAN speed at 500 kbps
//#include <Canbus.h> // only used CANSPEED_xxx above
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include "git-version.h"

//***************CAN IDs**************
#define RC_CMD_CAN_ID 0x101
#define ENC_CAN_ID 0x105
#define BUMP_CAN_ID 0x121
#define TEST_CAN_ID 0x321
#define GYRO_CAN_ID 0x131
#define ACCEL_CAN_ID 0x132
#define COMPASS_CAN_ID 0x133
#define BATT_CAN_ID 0x140

//**************Bumper, Battery*********
#define BUMP_PIN 8
#define BATT_PIN A2
//**************RC signals IN*********
#define AUTO_PIN 5
#define SPEED_PIN 6
#define STEER_PIN 7
//**************Encoders*************
#include <PinChangeInt.h>
#define RIGHT_ENC_A 3
#define RIGHT_ENC_B 4
#define LEFT_ENC_A 14 //A0
#define LEFT_ENC_B 15 //A1
//********************************Setup Loop*********************************//

long timeRC, timeGyro, timeHeading, timeBatt;
long prev_enc_left, prev_enc_right;
int prev_steer, prev_speed;
int dtheta = 0;

int16_t enc_left = 0, enc_right = 0;
int16_t steer_pwm = 1385, speed_pwm=1350, auto_pwm = 1300;
boolean readSpeed = true;

tCAN msg;

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
  Serial.begin(9600);
  Serial.println("wheele_ard_main version: " GIT_VERSION);
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
  
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    // Send CAN ERROR MESSAGE, Software Reset, continue, do NOT use while(1)
    while(1);
  }
    
  delay(200);
  bno.setExtCrystalUse(true);
  timeRC = millis();
  timeGyro = millis();
  timeBatt = millis();
  timeHeading = millis();
}

//********************************Main Loop*********************************//

void loop() 
{
  if(millis() - timeBatt > 5000)
  {
    int16_t raw_battery_signal = analogRead(BATT_PIN);
    tx_can(BATT_CAN_ID, raw_battery_signal, 0, 0, 0);
    //Serial.print("Raw Battery: ");
    //Serial.println(raw_battery_signal);
    timeBatt = millis();
  }
  
  if(millis() - timeGyro > 50)
  {
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    /*Serial.print("X: ");
    Serial.print(gyro.x());
    Serial.print(" Y: ");
    Serial.print(gyro.y());
    Serial.print(" Z: ");
    Serial.println(gyro.z());*/
    int16_t gyroXcentiDeg = int(gyro.x()*180.0/3.1416*100);
    int16_t gyroYcentiDeg = int(gyro.y()*180.0/3.1416*100);
    int16_t gyroZcentiDeg = int(gyro.z()*180.0/3.1416*100);
    //Serial.println(gyroZcentiDeg);
    tx_can(GYRO_CAN_ID, gyroXcentiDeg, gyroYcentiDeg, gyroZcentiDeg, 0);
    tx_can(ENC_CAN_ID, enc_left, enc_right, 0, 0);
    timeGyro = millis();
  }
  
  if(millis() - timeHeading > 100)
  {
    sensors_event_t event; 
    bno.getEvent(&event);
    float heading = (float)event.orientation.x;
    //float y = (float)event.orientation.y;
    //float z = (float)event.orientation.z;
    /*Serial.print("orientation x, y, z: ");
    Serial.print(heading );
    Serial.print(", ");
    Serial.print(y );
    Serial.print(", ");
    Serial.println(z );*/
    if(heading > 180.0)
    {
        heading -= 360.0;
    }
    int16_t headingCentiDeg = int(-heading*100.0); //centi-deg +/-180 deg
    
    imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    int16_t magx_uTx100 = int(magnet.x()*100); //uTx100
    int16_t magy_uTx100 = int(magnet.y()*100); //uTx100
    int16_t magz_uTx100 = int(magnet.z()*100); //uTx100
    
    tx_can(COMPASS_CAN_ID, magx_uTx100, magy_uTx100, magz_uTx100, headingCentiDeg); //Do NOT trust bno055 heading, auto-cal too dynamic
    
    imu::Vector<3> grav_accel = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    int16_t temp_ax = int(grav_accel.x()*100);
    int16_t temp_ay = int(grav_accel.y()*100);
    int16_t temp_az = int(grav_accel.z()*100);
    
    tx_can(ACCEL_CAN_ID, temp_ax, temp_ay, temp_az, 0);
    timeHeading = millis();
  }
  
  //SEND RAW RC CMDS
  if(millis() - timeRC > 100)
  {
    auto_pwm = pulseIn(AUTO_PIN, HIGH, 15000);
    if(auto_pwm < 1600)
    {
      if(readSpeed)
      {
        speed_pwm = pulseIn(SPEED_PIN, HIGH, 15000);
      }
      else
      {
        steer_pwm = pulseIn(STEER_PIN, HIGH, 15000);
      }
      readSpeed = !readSpeed;
      //speed_pwm = pulseIn(SPEED_PIN, HIGH, 15000);
      //steer_pwm = pulseIn(STEER_PIN, HIGH, 15000);
      tx_can(RC_CMD_CAN_ID, speed_pwm, steer_pwm, auto_pwm, 0);
      if(speed_pwm < 100)
      {Serial.print("speed rc: "); Serial.println(speed_pwm);}
      if(steer_pwm < 100)
      {Serial.print("steer rc: "); Serial.println(steer_pwm);}
      if(auto_pwm < 100)
      {Serial.print("auto rc: "); Serial.println(auto_pwm);}
      
    }
    else
    {
      tx_can(RC_CMD_CAN_ID, 1350, 1385, auto_pwm, 0);
    }
    int16_t bumper = digitalRead(BUMP_PIN);
    
    tx_can(BUMP_CAN_ID, bumper, 0, 0, 0);
    //tx_can(TEST_CAN_ID, 1350, 1500, 0, 0);
    timeRC = millis();
  }
  
  /*if(rx_can(&msg))
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
  }*/

}

uint8_t tx_can(uint16_t id, int16_t num1, int16_t num2, int16_t num3, int16_t num4)
{
  uint16_t raw1 = num1 + 32768;
  uint16_t raw2 = num2 + 32768;
  uint16_t raw3 = num3 + 32768;
  uint16_t raw4 = num4 + 32768;
  
  tCAN message;

  message.id = id; //0x631; //formatted in HEX
  message.header.rtr = 0;
  message.header.length = 8; //formatted in DEC
  message.data[0] = getByte(raw1,1);
  message.data[1] = getByte(raw1,0);
  message.data[2] = getByte(raw2,1);
  message.data[3] = getByte(raw2,0);
  message.data[4] = getByte(raw3,1);
  message.data[5] = getByte(raw3,0);
  message.data[6] = getByte(raw4,1);
  message.data[7] = getByte(raw4,0);
  
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
  return (message.data[start_byte] << 8 ) + (message.data[start_byte+1]) - 32768;
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
