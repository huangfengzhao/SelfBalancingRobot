#include "PID_v1.h"
#include "LMotorController.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <Wire.h>

//#define LOG_INPUT true

#define MIN_ABS_SPEED           100
#define MIN_ABS_FALLING_ATTI    0.45
#define MIN_FALLING_GRAVITY     0.75

MPU6050 mpu;

// MPU control/status variables.
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO

// Orientation/motion variables.
Quaternion quaternion;  // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Boolean flag indicating if the robot is falling down.
bool isRobotFallDown = true;

//PID
// === Robot #1
//double originalSetpoint = 182.8;
// === Robot #2
double originalSetpoint = 182.5;
double setpoint = originalSetpoint;
double input, output;

// Chosen PID:
//    Kp      Kd      Ki
//   --------------------
//    150     2.2     80  (loading with iphone8)
//
// === Robot #1
//double Kp = 96;
//double Kd = 2.8;
//double Ki = 80;
// === Robot #2
double Kp = 86;
double Kd = 3.2;
double Ki = 80;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.6;
double motorSpeedFactorRight = 0.65;

// L298N Motor controller connection.
int ENA = 0;
int IN1 = 2;
int IN2 = 14;
int IN3 = 12;
int IN4 = 13;
int ENB = 15;
int INT_IN = 10;

LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

// Boolean flag indicating whether or not it is interrupted from MPU.
volatile bool mpuInterrupted = false;
void ICACHE_RAM_ATTR dmpDataReady()
{
    mpuInterrupted = true;
}

void setup()
{
    Wire.begin();

    // initialize serial communication
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // Initialize the interrupt input, mode INPUT or INPUT_PULLUP
    pinMode(INT_IN, INPUT_PULLUP); // OR, mode: INPUT

    // initialize device
    Serial.println("Initializing I2C devices...");
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    uint8_t status = mpu.dmpInitialize();
    // make sure it worked (returns 0 if so)
    if (status == 0)
    {
        // Supply your own gyro offsets here, scaled for min sensitivity
        // These parameters can be calibrated thru IMU_Zero.ino, which is coming with MPU6050.
        // ==== Robot #1
        //mpu.setXGyroOffset(24);
        //mpu.setYGyroOffset(40);
        //mpu.setZGyroOffset(-24);
        //mpu.setZAccelOffset(1446);
        // ==== Robot #2
        mpu.setXGyroOffset(280);
        mpu.setYGyroOffset(44);
        mpu.setZGyroOffset(-96);
        mpu.setZAccelOffset(1403);

        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt)..."));

        attachInterrupt(digitalPinToInterrupt(INT_IN), dmpDataReady, RISING);

        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        //setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);  
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(status);
        Serial.println(F(")"));
    }
}

void loop()
{
    // Do nothing if initialization is failed.
    if (!dmpReady) return;

    // Wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupted && fifoCount < packetSize)
    {
        // Performing PID calculations and output to motors
        pid.Compute();
        if (!isRobotFallDown)
        {
            motorController.move(output, MIN_ABS_SPEED);
        }
    }

    // Reset interrupt flag and get INT_STATUS byte.
    mpuInterrupted = false;
    mpuIntStatus = mpu.getIntStatus();

    // Get current FIFO count.
    fifoCount = mpu.getFIFOCount();

    // Check for DMP data ready interrupt (this should happen frequently)
    if (mpuIntStatus & 0x02)
    {
        // Wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // Track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &quaternion);
        mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity);

        // Detect if robot is falling down.
        if(abs(ypr[1]) > MIN_ABS_FALLING_ATTI || abs(ypr[2]) > MIN_ABS_FALLING_ATTI || gravity.z < MIN_FALLING_GRAVITY)
        {
            isRobotFallDown = true;
            motorController.stopMoving();
        }
        else
        {
            isRobotFallDown = false;
        }

        #if LOG_INPUT

        Serial.print("gravity\t");
        Serial.print(gravity.x);
        Serial.print("\t");
        Serial.print(gravity.y);
        Serial.print("\t");
        Serial.print(gravity.z);
        Serial.print("\t");
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);
        #endif

        input = ypr[1] * 180/M_PI + 180;
   }
}
