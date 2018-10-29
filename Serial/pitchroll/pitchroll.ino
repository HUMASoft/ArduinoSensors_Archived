// MPU-9250 Accelerometer Example
// Jordan Day

/*
=========================================
This code is placed under the MIT license
Copyright (c) 2016 Jordan Day

Please reference the LICENSE file
=========================================
*/

#include "MPU9250.h"
#include "Wire.h"
#include "math.h"

MPU9250 imu;

int16_t accel[3];

void setup()
{
    Serial.begin(9600);
    imu.calibrate();
    imu.init();
}

void loop()
{
    float pitch, roll, fX, fY, fZ;
    int roll_t;

    imu.readAccelData(&accel[0]);

    fX = (float)accel[0] * imu.AccelRes;
    fY = (float)accel[1] * imu.AccelRes;
    fZ = (float)accel[2] * imu.AccelRes;

//    Print g force
//    Serial.print("X: ");
//    Serial.print(fX);
//    Serial.print(" Y: ");
//    Serial.print(fY);
//    Serial.print("   ");
//    Serial.println(fZ);

    pitch = (atan2(sqrt(fY * fY + fX * fX),fZ) * 180.0) / M_PI;

    //roll2 = (atan2(fY,fX) * 180.0) / M_PI;
    roll = (atan(fY/fX) * 180.0) / M_PI;
//    
    if(fX<0 && fY>0){//II Cuadrante
    roll= 180+roll;}
    
    if(fX<0 && fY<0){//III Cuadrante
    roll= 180+roll;}
    
    if(fX>0 && fY<0){//IV Cuadrante
    roll= 360+roll;}
    
    roll_t=(int)roll;
    
    Serial.print(roll_t);
    Serial.print(",");
    Serial.println((int8_t)pitch);
    Serial.print('\n');
    Serial.print('\n');

    //delay(100);
}
