/* ============================================
* Motorcycle Data logger sketch by Gideon Nyaga...
* giddyeffects@gmail.com
===============================================
*/

//==========INCLUDES==========
// for memory storage in program space
#include <avr/pgmspace.h> 
#include "I2Cdev.h"
//include time lib
#include "Time.h"
//include RTC lib
#include "DS1307RTC.h"
#include <elapsedMillis.h> //load the elasped library
#include "Wire.h"

//==========VARIABLES==========
//Create an elapsedMillis Instance
elapsedMillis timeElapsed;
//I2C address of the MPU6050.
const int MPU=0x69;  
const float alpha = 0.955; //Low Pass Filter smoothing factor
int16_t ax,ay,az,gx,gy,gz,airTemp;//was int
double pitch, roll, yaw, fXg=0, fYg=0, fZg=0, timer;
double accXangle ,accYangle,accZangle ,gyroXrate ,gyroYrate,gyroZrate;
double gyroXAngle, gyroYAngle, gyroZAngle;

void setup() {

    Serial.begin(115200);
    
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B); 
    Wire.write(0); 
    Wire.endTransmission(true);

}

void loop() {
    
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    
    //Requests the data from the sensor.
    Wire.requestFrom(MPU,14,true);  

  //Read the sensor data.
    ax = Wire.read()<<8|Wire.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    ay = Wire.read()<<8|Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    az = Wire.read()<<8|Wire.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    airTemp = Wire.read()<<8|Wire.read();  //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gx = Wire.read()<<8|Wire.read();  //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gy = Wire.read()<<8|Wire.read();  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gz = Wire.read()<<8|Wire.read();  //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    
    Serial.println("Raw Values");
    Serial.println("==========");
    Serial.print("ax/ay/az/gx/gy/gz/airTemp\t");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.print("\t");
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(gy);
    Serial.print("\t");
    Serial.print(gz);
    Serial.print("\t");
    Serial.print(airTemp/340 + 36.53);
    Serial.println();

  accXangle = (atan2(ay, az) * RAD_TO_DEG);
  accYangle = (atan2(ax, az) * RAD_TO_DEG);
  accZangle = (atan2(ax,ay) * RAD_TO_DEG);/* calculate yaw but not correct*/
  gyroXrate = gx / 16.5;
  gyroYrate = gy / 16.5;
  gyroZrate = gz / 16.5;
  timer = millis();
  //angular position
  gyroXAngle += gyroXrate * (millis()-timer)/1000;
  gyroYAngle += gyroYrate * (millis()-timer)/1000;
  gyroZAngle += gyroZrate * (millis()-timer)/1000;/* alculate yaw but not correct*/

  roll = alpha * ( roll + gyroXAngle) + (1-alpha) * accXangle;
  pitch = alpha * (pitch + gyroYAngle) + (1-alpha) * accYangle;
  yaw = alpha * (yaw + gyroZAngle) + (1-alpha) * accZangle; /*yaw but not correct*/
 /* first attempt
    //Low Pass filter
    fXg = gx * alpha + (fXg * (1.0 - alpha));
    fYg = gy * alpha + (fYg * (1.0 - alpha));
    fZg = gz * alpha + (fZg * (1.0 - alpha));

    //Roll & Pitch Equations
    roll  = (atan2(-fYg, fZg)*180.0)/M_PI;
    pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI; */
    Serial.print(yaw);
    Serial.print(":");
    Serial.print(pitch);
    Serial.print(":");
    Serial.println(roll);
  delay(100);
    
}
