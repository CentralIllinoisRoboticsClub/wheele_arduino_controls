/****************************************************************************
Based on tutorial by Stephen McCoy. 
Original tutorial available here: http://www.instructables.com/id/CAN-Bus-Sniffing-and-Broadcasting-with-Arduino
Used with permission 2016. License CC By SA. 

Distributed as-is; no warranty is given.
*************************************************************************/

#include <mcp_can.h>
#include <SPI.h>
#include <Servo.h>

const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

// Define wheelbase in meters
#define WHEELBASE_WIDTH (23.0 * 2.54 / 100)
#define WHEELBASE_LENGTH (12.5 * 2.54 / 100)
// Define maximum (full throttle) speed in m/s
#define MAX_SPEED 16.0
// Define maximum curvature in 1/m.  A curvature of 2.0 corresponds to turning radius of 0.5 meter.
#define MAX_CURVATURE 2.0

//***************CAN IDs**************
#define RC_CMD_CAN_ID 0x101
#define ENC_CAN_ID 0x105
#define BUMP_CAN_ID 0x121
#define TEST_CAN_ID 0x321
#define GYRO_CAN_ID 0x131
#define ACCEL_CAN_ID 0x132
#define COMPASS_CAN_ID 0x133
#define POWER_CAN_ID 0x140
#define RAW_THRUST_CAN_ID 0x301
#define RAW_STEER_CAN_ID 0x302

//**************Bumper*********
#define BUMP_PIN A0

//**************Power Inputs*********
#define BATT_VOLTAGE_PIN A6
#define BATT_CURRENT_PIN A7

//**************Encoders*************
#include <PinChangeInt.h>
#define RIGHT_ENC_A A2
#define RIGHT_ENC_B A3
#define LEFT_ENC_A A4
#define LEFT_ENC_B A5

//*************Servos/ESCs***
#define RL_SERVO 4
#define RR_SERVO 5
#define FR_SERVO 6
#define FL_SERVO 7
#define RIGHT_ESC 8
#define LEFT_ESC 9
//*************Data Frequency, Periods*********
#define SERVO_CHECK_PERIOD 1000  
#define BATTERY_PERIOD 1000
#define ENC_PERIOD 50 //msec
#define BUMP_PERIOD 100 //msec

//********************************Global Variables*********************************//

uint16_t timeBump, timeEnc, timeServoCheck, timeBatt;
long prev_enc_left, prev_enc_right;

int16_t enc_left = 0, enc_right = 0;

bool autoMode = false;

Servo leftESC;
Servo rightESC;
Servo frontLeftSteerServo;
Servo frontRightSteerServo;
Servo rearLeftSteerServo;
Servo rearRightSteerServo;


//********************************Setup Loop*********************************//

void setup() {
  Serial.begin(115200);
  //Serial.println("CAN Write - Testing transmission of CAN Bus messages");
  delay(200);

  while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 250K
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
  attachPinChangeInterrupt(RIGHT_ENC_A, right_enc_tick, CHANGE);
  attachPinChangeInterrupt(LEFT_ENC_A, left_enc_tick, CHANGE);
  
  delay(200);

  // Attach the servos
  leftESC.attach(LEFT_ESC);
  rightESC.attach(RIGHT_ESC);
  frontLeftSteerServo.attach(FL_SERVO);
  frontRightSteerServo.attach(FR_SERVO);
  rearLeftSteerServo.attach(RL_SERVO);
  rearRightSteerServo.attach(RR_SERVO);

  timeBump = millis();
  timeEnc = millis();
  timeServoCheck = millis();
}

//********************************Main Loop*********************************//

void loop() 
{
  static uint16_t currentMin = 65535;
  static uint16_t currentMax = 0;
  static uint32_t currentSum = 0;
  static uint32_t voltageSum = 0;
  static uint16_t analogNsamp = 0;
  uint16_t aIn;

  // Track min, max, and mean current readings; and mean voltage
  aIn = analogRead(BATT_CURRENT_PIN);
  currentSum += aIn;
  if (aIn > currentMax) currentMax = aIn;
  if (aIn < currentMin) currentMin = aIn;
  aIn = analogRead(BATT_VOLTAGE_PIN);
  voltageSum += aIn;
  ++analogNsamp;
  
  if(CAN_MSGAVAIL == CAN.checkReceive())
  {
    unsigned int rxId;
    unsigned char len = 0;
    unsigned char msg[8];
    // MUST CALL readMsgBuf BEFORE getCanId()
    CAN.readMsgBuf(&len, msg);
    rxId= CAN.getCanId();
    if(rxId == RC_CMD_CAN_ID)
    {
      // Received a servo RC packet.  Reset the timer and parse the packet.
      timeServoCheck = millis();
      parseRCCmd(msg);
    }
    else if(rxId == RAW_THRUST_CAN_ID)
    {
      // Received a thrust packet.  Parse the packet.
      parseThrustCmd(msg);
    }
    else if(rxId == RAW_STEER_CAN_ID)
    {
      // Received a steer packet.  Parse the packet.
      parseSteerCmd(msg);
    }
  }

  if(timeSince(timeServoCheck) > SERVO_CHECK_PERIOD)
  {
    // After timeout with no CAN RC packet, send STOP pulses to the ESCs
    leftESC.writeMicroseconds(1500);
    rightESC.writeMicroseconds(1500);
    frontLeftSteerServo.writeMicroseconds(1500);
    frontRightSteerServo.writeMicroseconds(1500);
    rearLeftSteerServo.writeMicroseconds(1500);
    rearRightSteerServo.writeMicroseconds(1500);
    autoMode = false;
    timeServoCheck = millis();
  }

  if (timeSince(timeBatt) > BATTERY_PERIOD)
  {
    if (analogNsamp > 0)
    {
      int16_t meanVoltage_mV;
      int16_t meanCurrent_mA;
      int16_t minCurrent_mA;
      int16_t maxCurrent_mA;
      // The voltage input is the raw battery divided by 3.  Convert the raw A/D to volts by
      // multiplying by 5/1024.  Divide by 3 to get battery volts.  Multiply by 1000 to get
      // millivolts.  The overall scaling is 5 * 1000 / (3 * 1024).
      meanVoltage_mV = 
        ((voltageSum + analogNsamp/2) / analogNsamp)  // Mean raw A/D value
        * 5 * 1000 / (1024 * 3);
      // The current input is scaled by ????
      meanCurrent_mA = 
        ((currentSum + analogNsamp/2) / analogNsamp)  // Mean raw A/D value
        ;
      minCurrent_mA = currentMin;
      maxCurrent_mA = currentMax;
      tx_can(POWER_CAN_ID, meanVoltage_mV, meanCurrent_mA, minCurrent_mA, maxCurrent_mA);
      // Reset statistics for next time
      analogNsamp = 0;
      voltageSum = currentSum = 0;
      currentMin = 65535;
      currentMax = 0;
    }
    timeBatt = millis();
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
  data[0] = raw1 >> 8;
  data[1] = raw1 & 0xFF;
  data[2] = raw2 >> 8;
  data[3] = raw2 & 0xFF;
  data[4] = raw3 >> 8;
  data[5] = raw3 & 0xFF;
  data[6] = raw4 >> 8;
  data[7] = raw4 & 0xFF;
  
  CAN.sendMsgBuf(id,0, 8, data); //id, 0 means NOT extended ID
  return 0;
  
}

int16_t convertCAN(unsigned char data[], unsigned int start_byte) //start byte is 0 to 7, left to right
{
  return (data[start_byte] << 8 ) + (data[start_byte+1]) - 32768;
}

void parseRCCmd(unsigned char * data)
{
  uint16_t speedPwm, steerPwm, autoPwm;
  float spd, curvature, dx, dy, spdLeft, spdRight, steerLeft, steerRight, absSpd;
#if 0  
  Serial.print(data[0]); Serial.print(", ");
  Serial.print(data[1]); Serial.print(", ");
  Serial.print(data[2]); Serial.print(", ");
  Serial.print(data[3]); Serial.print(", ");
  Serial.print(data[4]); Serial.print(", ");
  Serial.print(data[5]); Serial.print(", ");
  Serial.print(data[6]); Serial.print(", ");
  Serial.println(data[7]);
#endif

  autoPwm = ((uint16_t)data[4] << 8) + data[5] - 32768;
  steerPwm = ((uint16_t)data[0] << 8) + data[1] - 32768;
  speedPwm = ((uint16_t)data[2] << 8) + data[3] - 32768;
#if 0
  Serial.print(autoPwm); Serial.print(", ");
  Serial.print(speedPwm); Serial.print(", ");
  Serial.println(steerPwm);
#endif

  // Enter auto mode if autoPwm > 1700; leave auto mode if autoPwm < 1600.  100us hysteresis.
  if (autoMode)
    autoMode = (autoPwm > 1600);
  else
    autoMode = (autoPwm > 1700);

  if (!autoMode)
  {
    // Manual operation mode.  Get speed and curvature.
    spd = (speedPwm - 1500.0) / 500.0;
    curvature = (steerPwm - 1500.0) * MAX_CURVATURE / 500;
    // Calculate left/right speeds in m/s and wheel angles in radians
    dy = curvature * WHEELBASE_LENGTH / 2;
    dx = 1 + curvature * WHEELBASE_WIDTH / 2;
    spdLeft = spd * sqrt(dx * dx + dy * dy);
    steerLeft = 90.0 + atan(dy / dx) * 180.0 / 3.1416;
    dx = 1 - curvature * WHEELBASE_WIDTH / 2;
    spdRight = spd * sqrt(dx * dx + dy * dy);
    steerRight = 90.0 + atan(dy / dx) * 180.0 / 3.1416;
    // Adjust speed left/right if needed to keep in range +/-1.0
    absSpd = fabs(spdRight);
    if (absSpd < 1.0)
      absSpd = fabs(spdLeft);
    if (absSpd > 1.0) 
    {
      spdRight /= absSpd;
      spdLeft /= absSpd;
    }
#if 0
  Serial.print(spd); Serial.print(", ");
  Serial.print(curvature); Serial.print(", ");
  Serial.print(spdLeft); Serial.print(", ");
  Serial.println(spdRight);
  Serial.print(spdLeft * 500 + 1500); Serial.print(", ");
  Serial.print(spdRight * 500 + 1500); Serial.print(", ");
  Serial.print(steerLeft); Serial.print(", ");
  Serial.println(steerRight);
#endif
    // Convert speeds to pwm and output
    leftESC.writeMicroseconds(spdLeft * 500 + 1500);
    rightESC.writeMicroseconds(spdRight * 500 + 1500);
    // Convert angles to pwm and output
    frontLeftSteerServo.write(steerLeft);
    frontRightSteerServo.write(steerRight);
    rearLeftSteerServo.write(180.0 - steerLeft);
    rearRightSteerServo.write(180.0 - steerRight);
  }
}

void parseThrustCmd(unsigned char * data)
{
  uint16_t leftESCpwm;
  uint16_t rightESCpwm;

  // Only honor thrust commands if in auto mode
  if (autoMode)
  {
    leftESCpwm = ((uint16_t)data[0] << 8) + data[1];
    rightESCpwm = ((uint16_t)data[2] << 8) + data[3];
    leftESC.writeMicroseconds(leftESCpwm);
    rightESC.writeMicroseconds(rightESCpwm);
  }
}

void parseSteerCmd(unsigned char * data)
{
  uint16_t frontLeftServoPwm;
  uint16_t frontRightServoPwm;
  uint16_t rearLeftServoPwm;
  uint16_t rearRightServoPwm;

  // Only honor servo commands if in auto mode
  if (autoMode)
  {
    frontLeftServoPwm = ((uint16_t)data[0] << 8) + data[1];
    frontRightServoPwm = ((uint16_t)data[2] << 8) + data[3];
    rearLeftServoPwm = ((uint16_t)data[4] << 8) + data[5];
    rearRightServoPwm = ((uint16_t)data[6] << 8) + data[7];
    frontLeftSteerServo.writeMicroseconds(frontLeftServoPwm);
    frontRightSteerServo.writeMicroseconds(frontRightServoPwm);
    rearLeftSteerServo.writeMicroseconds(rearLeftServoPwm);
    rearRightSteerServo.writeMicroseconds(rearRightServoPwm);
  }
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
