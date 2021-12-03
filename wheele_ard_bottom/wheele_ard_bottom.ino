/****************************************************************************
Based on tutorial by Stephen McCoy. 
Original tutorial available here: http://www.instructables.com/id/CAN-Bus-Sniffing-and-Broadcasting-with-Arduino
Used with permission 2016. License CC By SA. 

Distributed as-is; no warranty is given.
*************************************************************************/

/****************************************************************************
** Planned updates:
** 1. Add closed loop speed control based on odometer reading
** 2. For steering servos, use writeMicroseconds() instead of write()
** 3. Remove TURN_IN_PLACE_VELOCITY and replace with calculated value
*************************************************************************/

#include <mcp_can.h>
#include <SPI.h>
#include <Servo.h>

const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

// Define wheelbase in meters
#define WHEELBASE_WIDTH (23.0 * 2.54 / 100)
#define WHEELBASE_LENGTH (12.5 * 2.54 / 100)
// Define maximum curvature in 1/m.  A curvature of 2.0 corresponds to turning radius of 0.5 meter.
#define MAX_CURVATURE 2.0
// Define the velocity for turning in place, in m/s
//#define TURN_IN_PLACE_VELOCITY 1.0
// Define maximum velocity in m/s
#define MAX_VELOCITY 2.0

// Scaling for encoder ticks
#define ENC_METERS_PER_TICK (1.0 / 900.0)

//***************CAN IDs**************
#define RC_CMD_CAN_ID 0x101
#define ENC_CAN_ID 0x105
#define BUMP_CAN_ID 0x121
#define TEST_CAN_ID 0x321
#define GYRO_CAN_ID 0x131
#define ACCEL_CAN_ID 0x132
#define COMPASS_CAN_ID 0x133
#define POWER_CAN_ID 0x140
#define CMD_VEL 0x301

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
#define RL_SERVO 9
#define RR_SERVO 8
#define FL_SERVO 7
#define FR_SERVO 6
#define RIGHT_ESC 5
#define LEFT_ESC 4
//*************Steering Servo Calibration************
//#define RL_SERVO_OFFSET (40 * 180.0 / 1000.0)
//#define RR_SERVO_OFFSET (120 * 180.0 / 1000.0)
//#define FL_SERVO_OFFSET (100 * 180.0 / 1000.0)
//#define FR_SERVO_OFFSET (0 * 180.0 / 1000.0)
#define SERVO_US_PER_DEG 17.0
#define FL_SERVO_MIN    1250
#define FL_SERVO_CENTER 1600
#define FL_SERVO_MAX    2000
#define FR_SERVO_MIN    1030
#define FR_SERVO_CENTER 1500
#define FR_SERVO_MAX    1840
#define RL_SERVO_MIN    1150
#define RL_SERVO_CENTER 1540
#define RL_SERVO_MAX    1900
#define RR_SERVO_MIN    1300
#define RR_SERVO_CENTER 1620
#define RR_SERVO_MAX    2000
//*************Data Frequency, Periods*********
#define SERVO_CHECK_PERIOD 1000  
#define BATTERY_PERIOD 1000
#define ENC_PERIOD 50 //msec
#define BUMP_PERIOD 100 //msec

//*************Velocity tuning gains*********
#define KF 250.0    // Feedforward:  1m/s results in 50% power to motors
#define KP 0.0    // Proportional:  No proportional gain
#define KI 50.0     // Integral: 1m/s error results in 0 to 100% ramp of 1 second with 100ms control loop

//********************************Global Variables*********************************//

uint16_t timeBump, timeEnc, timeServoCheck, timeBatt, timeServoUpdate;
float measuredVelocityLeft, measuredVelocityRight;

int16_t encLeft = 0, encRight = 0;
int16_t prevEncLeft = 0, prevEncRight = 0;

bool autoMode = false;

// For WheelE ESCs, lower pulse widths result in forward motion
Servo leftESC;
Servo rightESC;
// For WheelE servos, on all 4 wheels, lower pulse widths turn the wheels CCW
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
  timeServoUpdate = millis();

#if 0
while (1)
{
  updateServos(0.0, 0.0);
  delay(2000);
//  updateServos(0.5, 2.0);
  delay(2000);
//  updateServos(0.25, 1.0);
  delay(2000);
}
#endif
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
    else if(rxId == CMD_VEL)
    {
      // Received a thrust packet.  Parse the packet.
      parseCmdVel(msg);
    }
  }

  if(timeSince(timeServoCheck) > SERVO_CHECK_PERIOD)
  {
    // After timeout with no CAN RC packet, send STOP pulses to the ESCs
    leftESC.writeMicroseconds(1500);
    rightESC.writeMicroseconds(1500);
    frontLeftSteerServo.writeMicroseconds(FL_SERVO_CENTER);
    frontRightSteerServo.writeMicroseconds(FR_SERVO_CENTER);
    rearLeftSteerServo.writeMicroseconds(RL_SERVO_CENTER);
    rearRightSteerServo.writeMicroseconds(RR_SERVO_CENTER);
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
    tx_can(ENC_CAN_ID, encLeft, encRight, 0, 0);
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
      Serial.print(encLeft); Serial.print(", ");
      Serial.println(encRight);
    }
  }*/

}

//*************************************************************************************
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

//*************************************************************************************
int16_t convertCAN(unsigned char data[], unsigned int start_byte) //start byte is 0 to 7, left to right
{
  return (data[start_byte] << 8 ) + (data[start_byte+1]) - 32768;
}

//*************************************************************************************
void parseRCCmd(unsigned char * data)
{
  uint16_t speedPwm;
  uint16_t steerPwm;
  uint16_t autoPwm;
  float velocity;
  float curvature;
  
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

  // Limit pulse widths to range [1000, 2000] microseconds
  if (autoPwm < 1000) autoPwm = 1000;
  else if (autoPwm > 2000) autoPwm = 2000;
  if (steerPwm < 1000) steerPwm = 1000;
  else if (steerPwm > 2000) steerPwm = 2000;
  if (speedPwm < 1000) speedPwm = 1000;
  else if (speedPwm > 2000) speedPwm = 2000;

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
    // Manual operation mode.  Get speed and curvature, and update the servo outputs.
    // 25us deadband
    if ((steerPwm > 1475) && (steerPwm < 1525))
      steerPwm = 1500;
    if ((speedPwm > 1475) && (speedPwm < 1525))
    {
      speedPwm = 1500;  // If both speed and steer are in deadband, do nothing.
    }
    // Compute speed and curvature.  Positive curvature is defined as CCW, which corresponds
    // to steerPwm less than 1500.
    velocity = (speedPwm - 1500.0) * MAX_VELOCITY / 500.0;
    curvature = (steerPwm - 1500.0) * MAX_CURVATURE / 500;
    
    updateServos(velocity, curvature);
  }
}

//*************************************************************************************
void parseCmdVel(unsigned char * data)
{
  float curvature;
  float velocity;
  
  // Only honor CmdVel commands if in auto mode
  if (!autoMode)
    return;

  // Get velocity and yaw rate from command
  int16_t velMmPerSec = ((uint16_t)data[0] << 8) + data[1] - 32768;
  int16_t yawRateDeciDeg = ((uint16_t)data[2] << 8) + data[3] - 32768;
  
  // Convert to floating point and standard units
  velocity = velMmPerSec / 1000.0;                   // meters/second
  float yawRate = yawRateDeciDeg * 3.14159 / 1800.0;      // radians/second

#if 0
  Serial.print("Vel mm/s = ");
  Serial.print(velMmPerSec);
  Serial.print(", Yaw deciDeg/sec = ");
  Serial.println(yawRateDeciDeg);
  Serial.print("Vel m/s = ");
  Serial.print(velocity);
  Serial.print(", Yaw rad/sec = ");
  Serial.println(yawRate);
#endif
  
  // Handle case where velocity is 0.  Use integer value for equality comparison.
  if (velMmPerSec == 0)
  {
    // If velocity and yaw are both zero, then curvature is zero
    if (yawRateDeciDeg == 0)
      curvature = 0.0;
    else
    {
      // If velocity is zero but yaw is non-zero, this indicates an attempt
      // to turn in place, which Wheel-E cannot do.  Turn in the direction
      // indicated as sharply as possible.
      if (yawRate > 0.0)
        curvature = MAX_CURVATURE;
      else
        curvature = -MAX_CURVATURE;
      // Compute velocity needed to achieve desired yaw rate
      velocity = yawRate / curvature;
      // Limit velocity to the maximum
      if (velocity > MAX_VELOCITY)
        velocity = MAX_VELOCITY;
      else if (velocity < -MAX_VELOCITY)
        velocity = -MAX_VELOCITY;
    }
  }
  else
  {
    // Velocity is non-zero, so calculate curvature.
    // Limit velocity to the maximum
    if (velocity > MAX_VELOCITY)
      velocity = MAX_VELOCITY;
    else if (velocity < -MAX_VELOCITY)
      velocity = -MAX_VELOCITY;
    // Velocity is non-zero, so compute the curvature
    curvature = yawRate / velocity;
    if (curvature > MAX_CURVATURE)
      curvature = MAX_CURVATURE;
    else if (curvature < -MAX_CURVATURE)
      curvature = -MAX_CURVATURE;
  }

#if 0
  Serial.print("Vel m/s = ");
  Serial.print(velocity);
  Serial.print(", Curvature 1/m = ");
  Serial.println(curvature);
#endif
  
  // Send commands to ESCs and servos to move according to the calculated velocity and curvature.
  updateServos(velocity, curvature);
}

//*************************************************************************************
void updateServos(float cmdVelocity, float cmdCurvature)
{
  float cdxL, cdxR, cdy;  // Delta x & y for curvature calculations
  float spdLeft, spdRight;
  float errorLeft, errorRight;
  float steerLeft, steerRight;
  uint16_t pw;
  static int16_t escLus, escRus;
  static float prevSpdLeft, prevSpdRight, prevErrorLeft, prevErrorRight;

#if 0
  Serial.print("updateServos:  v = "); Serial.print(cmdVelocity); Serial.print(", c = "); Serial.println(cmdCurvature);
#endif

  // Calculate left/right speeds in m/s and wheel angles in radians
  cdy = cmdCurvature * WHEELBASE_LENGTH / 2;
  cdxL = 1.0 - cmdCurvature * WHEELBASE_WIDTH / 2;
  
  spdLeft = sqrt(cdxL * cdxL + cdy * cdy) * cmdVelocity;
  steerLeft = atan(cdy / cdxL) * 180.0 / 3.1416;

  cdxR = 1.0 + cmdCurvature * WHEELBASE_WIDTH / 2;
  spdRight = sqrt(cdxR * cdxR + cdy * cdy) * cmdVelocity;
  steerRight = atan(cdy / cdxR) * 180.0 / 3.1416;
  
  // Speed left/right are in meters/second.  Convert to pulse widths in us based on tuning
  // parameters.  Use a PI control with feed-forward, in the derivative form.
  //   Standard form:  escLus = KF * spdLeft + KP * error + KI * errInt
  //   Derivative form:  escLus += KF * (spdLeft - prevSpdLeft) + KP * (error - prevError) + KI * error
  // The advantage of the derivative form is that it simplifies handling of integral runaway.
  float vScale = 1000.0 * ENC_METERS_PER_TICK / timeSince(timeServoUpdate);
  timeServoUpdate = millis();
  measuredVelocityLeft = (int16_t)(encLeft - prevEncLeft) * vScale;
  measuredVelocityRight = (int16_t)(encRight - prevEncRight) * vScale;
  prevEncLeft = encLeft;
  prevEncRight = encRight;
  errorLeft = spdLeft - measuredVelocityLeft;
  escLus += (int16_t)(KF * (spdLeft - prevSpdLeft)) + KP * (errorLeft - prevErrorLeft) + KI * errorLeft;
  prevSpdLeft = spdLeft;
  prevErrorLeft = errorLeft;
  errorRight = spdRight - measuredVelocityRight;
  escRus += (int16_t)(KF * (spdRight - prevSpdRight)) + KP * (errorRight - prevErrorRight) + KI * errorRight;
  prevSpdRight = spdRight;
  prevErrorRight = errorRight;
Serial.print(errorLeft);
Serial.print(", ");
Serial.println(errorRight);
//Serial.print("errL = "); Serial.print(errorLeft);
//Serial.print(", errR = "); Serial.println(errorRight);
//Serial.print("R us = "); Serial.println(escRus);
  
  // The speeds are now in signed microseconds centered around 0us, where positive is forward.
  // Limit to the range +/-500us.
  // NOTE:  This is the step that handles integral runaway.  The clipped speeds will be used as the
  // starting point for the next PI calculation, so the integral can never run away.
  if (escLus > 500) escLus = 500;
  else if (escLus < -500) escLus = -500;
  if (escRus > 500) escRus = 500;
  else if (escRus < -500) escRus = -500;
  
#if 0
  Serial.print("spdL = "); Serial.print(spdLeft); Serial.print(", spdR = "); Serial.println(spdRight);
  Serial.print("steerL = "); Serial.print(steerLeft); Serial.print(", steerR = "); Serial.println(steerRight);
#endif

  // Convert speeds to the range 1000 to 2000us and output.  
  leftESC.writeMicroseconds(1500 - escLus);
  rightESC.writeMicroseconds(1500 - escRus);
  
#if 0
  Serial.print("Left ESC us = ");
  Serial.print(escLus);
  Serial.print(", Right = ");
  Serial.println(escRus);
#endif

  // Convert angles to pwm and output.  Limit each servo to the
  // range it can reach without binding.  This range was determined
  // empirically.
  // Front left steering servo
  pw = FL_SERVO_CENTER - (uint16_t)(steerLeft * SERVO_US_PER_DEG);
  if (pw < FL_SERVO_MIN) pw = FL_SERVO_MIN;
  else if (pw > FL_SERVO_MAX) pw = FL_SERVO_MAX;
  frontLeftSteerServo.writeMicroseconds(pw);
//Serial.print("FL = ");
//Serial.print(pw);
  // Rear left steering servo
  pw = RL_SERVO_CENTER + (uint16_t)(steerLeft * SERVO_US_PER_DEG);
  if (pw < RL_SERVO_MIN) pw = RL_SERVO_MIN;
  else if (pw > RL_SERVO_MAX) pw = RL_SERVO_MAX;
  rearLeftSteerServo.writeMicroseconds(pw);
//Serial.print(", RL = ");
//Serial.print(pw);
  // Front right steering servo
  pw = FR_SERVO_CENTER - (uint16_t)(steerRight * SERVO_US_PER_DEG);
  if (pw < FR_SERVO_MIN) pw = FR_SERVO_MIN;
  else if (pw > FR_SERVO_MAX) pw = FR_SERVO_MAX;
  frontRightSteerServo.writeMicroseconds(pw);
//Serial.print(", FR = ");
//Serial.print(pw);
  // Rear right steering servo
  pw = RR_SERVO_CENTER + (uint16_t)(steerRight * SERVO_US_PER_DEG);
  if (pw < RR_SERVO_MIN) pw = RR_SERVO_MIN;
  else if (pw > RR_SERVO_MAX) pw = RR_SERVO_MAX;
  rearRightSteerServo.writeMicroseconds(pw);
//Serial.print(", RR = ");
//Serial.println(pw);
}

//*************************************************************************************
void left_enc_tick()
{
  // modify using PORT operations for efficiency
  if(digitalRead(LEFT_ENC_A) != digitalRead(LEFT_ENC_B))
  {
    encLeft--;
  }
  else
  {
    encLeft++;
  }
}

//*************************************************************************************
void right_enc_tick()
{
  // modify using PORT operations for efficiency
  if(digitalRead(RIGHT_ENC_A) != digitalRead(RIGHT_ENC_B))
  {
    encRight--;
  }
  else
  {
    encRight++;
  }
}

//*************************************************************************************
uint16_t timeSince(uint16_t startTime)
{
  return (uint16_t)(millis() - startTime);
}
