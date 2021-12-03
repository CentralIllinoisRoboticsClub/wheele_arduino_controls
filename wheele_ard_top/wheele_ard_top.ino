/****************************************************************************
Based on tutorial by Stephen McCoy. 
Original tutorial available here: http://www.instructables.com/id/CAN-Bus-Sniffing-and-Broadcasting-with-Arduino
Used with permission 2016. License CC By SA. 

Distributed as-is; no warranty is given.
*************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <EnableInterrupt.h>
#include <mcp_can.h>
#include "git-version.h"

const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

//***************CAN IDs**************
#define RC_CMD_CAN_ID 0x101
#define ENC_CAN_ID 0x105
#define BUMP_CAN_ID 0x121
#define TEST_CAN_ID 0x321
#define GYRO_CAN_ID 0x131
#define ACCEL_CAN_ID 0x132
#define COMPASS_CAN_ID 0x133
#define BATT_CAN_ID 0x140

//***************IMU Signals In**************
// NOTE:  The IMU inputs must be pins A4 and A5 because these are the hardware I2C pins
#define IMU_SDA A4
#define IMU_SCL A5

//**************RC Defines*********
// NOTE:  The CPPM pin must be D8 because the decoding uses the ICP function on D8
#define RC_CPPM_PIN 8
#define RC_CENTER 1480

#define RC_NUM_CHANNELS  6
#define RC_STEER 0
#define RC_SPEED 1
#define RC_LEFT_V 2 // Left joystick vertical movement, unused currently
#define RC_AUTO 3
#define SYNC_PULSE_THRESH 3000

#define RC_AUTO_PIN  4
#define RC_SPEED_PIN 5
#define RC_STEER_PIN 6

#define GYRO_PERIOD 50
#define HEADING_PERIOD 100
#define RC_PERIOD 100

volatile uint16_t rc_shared[RC_NUM_CHANNELS];
volatile uint8_t rc_channel;
int16_t steer_pwm = 1385;
int16_t speed_pwm=1350;
int16_t auto_pwm = 1300;
int16_t left_v_pwm;

long timeRC, timeGyro, timeHeading, timeBatt;
int prev_steer, prev_speed;

Adafruit_BNO055 bno = Adafruit_BNO055();
bool bno055Detected = false;

//*****************************************************************
// The servo pulses are read on a single pin using CPPM format, which
// encodes the pulses as time between falling edges.  Our receiver is
// an OrangeRX R617XL bound to a Spektrum DX4e Mode 2 radio.  The 6
// channels are mapped to 6 sequential pulses as follows:
// - Pulse #1: Steering (aileron control on aircraft)
// - Pulse #2: Speed (elevator control on aircraft)
// - Pulse #3: Unused (throttle control on aircraft)
// - Pulse #4: Auto (rudder control on aircraft)
// - Pulse #5: Unused (act/aux switch)
// - Pulse #6: Unused (trainer bind button)
// 
// The pulse widths are measured from falling edge to falling edge.
// Between groups of pulses is a synchronization signal, which is a
// pulse of approximately 10 to 15 milliseconds.  Valid pulses are
// between 1 and 2 milliseconds, so it is easy to distinguish between
// a sync pulse and a valid servo pulse.  This code uses 3ms as the
// threshold.  Any pulse greater than 3ms is considered to be a sync
// pulse.
//*****************************************************************
ISR (TIMER1_CAPT_vect)
{
  uint16_t captureValue;
  uint16_t pulseWidth;
  static uint16_t prevCaptureValue;

  // Calculate the pulse width in us from previous edge to this edge.  The timebase
  // for Timer1 is set to 2MHz, resulting in a 0.5us resolution.
  captureValue = ICR1;
  pulseWidth = (captureValue - prevCaptureValue) / 2; // Timer1 is 0.5us/count; convert to 1us/count
  prevCaptureValue = captureValue;

  // Detect sync pulse that starts a new set of servo pulses.  Anything greater than 3ms is considered
  // a sync pulse.
  if (pulseWidth >= SYNC_PULSE_THRESH)
  {
    rc_channel = 0;
    return;
  }

  // We only support 6 channels.  Return if more than 6 servo pulses are detected without a sync pulse.
  if (rc_channel >= RC_NUM_CHANNELS)
  {
    return;
  }

  // Store the pulse width at the appropriate index.
  rc_shared[rc_channel] = pulseWidth;
  ++rc_channel;
}  // end of TIMER1_CAPT_vect

//*****************************************************************
// Setup the CPPM input, using the Input Capture peripheral and interrupt.
//*****************************************************************
void cppmSetup(void)
{

  pinMode(RC_CPPM_PIN, INPUT_PULLUP);
  
  noInterrupts();

  // Init to invalid channel, which will cause the ISR to wait for a sync pulse
  rc_channel = 255;

  // Setup Timer1 for input capture, 0.5us resolution, ICP interrupt enabled
  TCCR1A = 0b00000000;  // No output compare or waveform generation
  TCCR1B = 0b00000010; // Input capture on falling edge; prescale /8
  bitSet(TIMSK1, ICIE1);  // Enable Input Capture interrupt
  
  interrupts();
}

//*****************************************************************
// Copy the RC pulse widths to an array.
//*****************************************************************
void rc_read_values() {
  noInterrupts();
  steer_pwm = rc_shared[RC_STEER];
  speed_pwm = rc_shared[RC_SPEED];
  auto_pwm = rc_shared[RC_AUTO];
  left_v_pwm = rc_shared[RC_LEFT_V];
  // Clear the pulses in the array after copying them.  This is used
  // to detect if the pulse train is not being received.
  rc_shared[RC_STEER] = rc_shared[RC_SPEED] = rc_shared[RC_AUTO] = rc_shared[RC_LEFT_V] = 1500;
  interrupts();
}

//*****************************************************************
// Handle the pin change interrupts for the PWM channels (when
// using separate PWM inputs instead of CPPM).
//*****************************************************************
void pinchange_rc_auto(void)
{
  static uint16_t prevMicros;
  if (digitalRead(RC_AUTO_PIN) == HIGH)
  {
    prevMicros = micros();
  }
  else
  {
    rc_shared[RC_AUTO] = (uint16_t)(micros() - prevMicros);
  }
}
void pinchange_rc_speed(void)
{
  static uint16_t prevMicros;
  if (digitalRead(RC_SPEED_PIN) == HIGH)
  {
    prevMicros = micros();
  }
  else
  {
    rc_shared[RC_SPEED] = (uint16_t)(micros() - prevMicros);
  }
}
void pinchange_rc_steer(void)
{
  static uint16_t prevMicros;
  if (digitalRead(RC_STEER_PIN) == HIGH)
  {
    prevMicros = micros();
  }
  else
  {
    rc_shared[RC_STEER] = (uint16_t)(micros() - prevMicros);
  }
}

//********************************Setup Loop*********************************//

void setup() {
  Serial.begin(115200);
  Serial.println("wheele_ard_top version: " GIT_VERSION);
  Serial.println("CAN Write - Testing transmission of CAN Bus messages");
  delay(200);
  

  while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_16MHz))
  {
      /*Serial.println("CAN BUS Module Failed to Initialized");
      Serial.println("Retrying....");*/
      delay(500);
      
  }

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    // Send CAN ERROR MESSAGE, Software Reset, continue, do NOT use while(1)
  }
  else
  {
    bno055Detected = true;
    Serial.println("BNO055 detected");
    delay(200);
    bno.setExtCrystalUse(true);
  }

  // Set up the CPPM input pin and interrupt.
  cppmSetup();

  // Set up the separate PWM inputs (alternate to CPPM).
  pinMode(RC_AUTO_PIN, INPUT_PULLUP);
  pinMode(RC_SPEED_PIN, INPUT_PULLUP);
  pinMode(RC_STEER_PIN, INPUT_PULLUP);
  enableInterrupt(RC_AUTO_PIN, pinchange_rc_auto, CHANGE);
  enableInterrupt(RC_SPEED_PIN, pinchange_rc_speed, CHANGE);
  enableInterrupt(RC_STEER_PIN, pinchange_rc_steer, CHANGE);

  timeRC = millis();
  timeGyro = millis();
  timeBatt = millis();
  timeHeading = millis();

  Serial.println("Setup complete");
}

//********************************Main Loop*********************************//

void loop() 
{
  if(bno055Detected && (timeSince(timeGyro) > GYRO_PERIOD))
  {
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
#if 0
    Serial.print("X: ");
    Serial.print(gyro.x());
    Serial.print(" Y: ");
    Serial.print(gyro.y());
    Serial.print(" Z: ");
    Serial.println(gyro.z());
#endif
    // Gyro values are returned in float degrees/second.  Convert to int16 centi degrees/sec.
    int16_t gyroXcentiDeg = int(gyro.x() * 100.0);
    int16_t gyroYcentiDeg = int(gyro.y() * 100.0);
    int16_t gyroZcentiDeg = int(gyro.z() * 100.0);
    //Serial.println(gyroZcentiDeg);
    tx_can(GYRO_CAN_ID, gyroXcentiDeg, gyroYcentiDeg, gyroZcentiDeg, 0);
#if 0
    // Display the mean dps over 20 readings (1 second)
    static float gxSum, gySum, gzSum;
    static uint8_t gyroMeanCount;
    gxSum += gyro.x();
    gySum += gyro.y();
    gzSum += gyro.z();
    ++gyroMeanCount;
    if (gyroMeanCount >= 20)
    {
      Serial.print("X: ");
      Serial.print(gxSum / 20.0);
      Serial.print(" Y: ");
      Serial.print(gySum / 20.0);
      Serial.print(" Z: ");
      Serial.println(gzSum / 20.0);
      gxSum = gySum = gzSum = 0.0;
      gyroMeanCount = 0;
    }
#endif

    timeGyro = millis();
  }
  
  if(bno055Detected && (timeSince(timeHeading) > HEADING_PERIOD))
  {
    sensors_event_t event; 
    bno.getEvent(&event);
    float heading = (float)event.orientation.x;
#if 0
    float y = (float)event.orientation.y;
    float z = (float)event.orientation.z;
    Serial.print("orientation x, y, z: ");
    Serial.print(heading );
    Serial.print(", ");
    Serial.print(y );
    Serial.print(", ");
    Serial.println(z );
#endif
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
  
  // Send RC pulse widths
  if(timeSince(timeRC) > RC_PERIOD)
  {
    rc_read_values(); // Get the RC values into global variables auto_pwm, speed_pwm, and steer_pwm
    tx_can(RC_CMD_CAN_ID, steer_pwm, speed_pwm, auto_pwm, left_v_pwm);
#if 0
    Serial.print("Speed: ");
    Serial.print(speed_pwm);
    Serial.print(" Steer: ");
    Serial.print(steer_pwm);
    Serial.print(" auto: ");
    Serial.println(auto_pwm);
#endif
    timeRC = millis();
  }

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

uint16_t timeSince(uint16_t startTime)
{
  return (uint16_t)(millis() - startTime);
}
