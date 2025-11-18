// ===============================================
// SELF-BALANCING ROBOT CODE (Anti-Windup Implemented)
// ===============================================

// *** Includes ***
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Check for I2C Implementation
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
#endif

#define MIN_ABS_SPEED 10

// MPU6050 GLOBALS (Unchanged)
MPU6050 mpu;
bool dmpReady = false; 
uint8_t mpuIntStatus; 
uint8_t devStatus; 
uint16_t packetSize; 
uint16_t fifoCount; 
uint8_t fifoBuffer[64]; 
Quaternion q;       
VectorFloat gravity; 
float ypr[3];        

// ===============================================
// *** PID CONTROL GLOBALS (REVISED FOR ANTI-WINDUP) ***
// ===============================================
// ** CRITICALLY IMPORTANT: MEASURE AND SET THIS VALUE **
double originalSetpoint = 176.65; // REPLACE with your measured upright angle! 
double setpoint = originalSetpoint;
double input, output;

// Custom variables for Anti-Windup (Integral Clamping)
double Kp = 80.0;     // HIGH P for strong reaction
double Kd = 3.0;      // D for damping
double Ki_manual = 0.07; // MANUALLY apply integral gain (low value)
double integralLimit = 1500.0; // The max accumulation of error allowed
double errorSum = 0.0; // The accumulator for the integral term
double lastInput = 0.0; // For Derivative calculation (rate of change)
unsigned long lastTime = 0; // For Sample Time calculation

// PID Controller Object (Ki set to 0.0, we calculate it manually)
// The library will only handle P and D now.
PID pid(&input, &output, &setpoint, Kp, 0.0, Kd, DIRECT); 

// Motor Controller Globals (Unchanged)
int ENA = 11; int IN1 = 7; int IN2 = 6;
int IN3 = 5; int IN4 = 4; int ENB = 10; 
double motorSpeedFactorLeft = 0.4;
double motorSpeedFactorRight = 0.4;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

// INTERRUPT HANDLER (Unchanged)
volatile bool mpuInterrupt = false; 
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  Serial.begin(115200);
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; 
  #endif
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(0); mpu.setYGyroOffset(0); mpu.setZGyroOffset(0); mpu.setZAccelOffset(0); 

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING); 
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    
    // Configure PID (Ki is 0.0 here)
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10); 
    pid.SetOutputLimits(-255, 255);
    
    // Initialize lastInput for the Derivative term
    lastInput = originalSetpoint;
  } else {
    Serial.print(F("DMP Init failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

// ===============================================
// *** MAIN LOOP FUNCTION (WITH MANUAL ANTI-WINDUP) ***
// ===============================================
void loop() {
  if (!dmpReady) return;

  if (!mpuInterrupt && fifoCount < packetSize) {
    return; 
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return;
  }
  
  // 1. Process Data and Update 'input'
  if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    input = ypr[1] * 180/M_PI + 180; 
    
    // Print the angle for monitoring
    Serial.print("Angle (Pitch): ");
    Serial.println(input);
    
    // 2. Custom PID Calculation for Anti-Windup
    unsigned long now = millis();
    double timeChange = (double)(now - lastTime);

    // Only compute if 10ms has passed (matching SetSampleTime)
    if (timeChange >= 10.0) {
      double error = setpoint - input;
      
      // I Term Calculation with CLAMPING
      errorSum += error * timeChange; // Accumulate error over time
      
      // *** ANTI-WINDUP IMPLEMENTATION ***
      if (errorSum > integralLimit) {
        errorSum = integralLimit;
      } else if (errorSum < -integralLimit) {
        errorSum = -integralLimit;
      }
      
      // D Term Calculation
      double dInput = (input - lastInput) / timeChange;
      
      // Manual PID Calculation
      double pTerm = Kp * error;
      double iTerm = Ki_manual * errorSum;
      double dTerm = -Kd * dInput; // Note: Damping should oppose rate of change
      
      // Combine and clamp the final output manually
      output = pTerm + iTerm + dTerm;
      
      // Clamp output to motor limits (-255 to 255)
      if (output > 255) output = 255;
      else if (output < -255) output = -255;

      // 3. Motor Output
      motorController.move(output, MIN_ABS_SPEED);
      
      // Update variables for next cycle
      lastInput = input;
      lastTime = now;
      
      Serial.print("PID Output: ");
      Serial.println(output);
    }
  }
}
