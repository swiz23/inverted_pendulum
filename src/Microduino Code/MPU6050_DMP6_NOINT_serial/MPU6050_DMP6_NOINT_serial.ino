#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;


bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void setup() {
    Serial1.begin(9600);
    Wire.begin();
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
   
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
}


void loop() {
    
    if (!dmpReady) return;

   
    fifoCount = mpu.getFIFOCount();
    if (fifoCount == 1024) {
        mpu.resetFIFO();
    } else{
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial1.println(ypr[2]* 180/M_PI);
    }
}
