/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.
 Modified by Brent Wilkins July 19, 2016

 Demonstrate basic MPU-9250 functionality including parameterizing the register
 addresses, initializing the sensor, getting properly scaled accelerometer,
 gyroscope, and magnetometer data out. Added display functions to allow display
 to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
 Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
 and the Teensy 3.1.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.

 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 */

#include "quaternionFilters.h"
#include "MPU9250.h"
#include "math.h"

#define AHRS false         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 13;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 12;  // Set up pin 13 led for toggling
float incl=0, orient=0;
float theta;
float phi;
int offset=0;
int n=0;
int dataRange[100];
float var, mean;
int sum=0;
int i=0;


#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

void setup()
{
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(9600);

  while(!Serial){};

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

   // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    //Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
   // Serial.println("..");
    
    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
 
    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    if (d != 0x48)
    {
      // Communication failed, stop here
      //Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
   // Serial.println("AK8963 initialized for active data mode....");

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    
    //myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    //write mag data
    
    //read mag data
    myIMU.magBias[0]=49.53;
    myIMU.magBias[1]=633.23;
    myIMU.magBias[2]=451.66;
    
//    Serial.println("AK8963 mag biases (mG)");
    Serial.print(myIMU.magBias[0]);
    Serial.print(",");
    Serial.print(myIMU.magBias[1]);
    Serial.print(",");
    Serial.println(myIMU.magBias[2]);
//

  } // if (c == 0x71)

}

void loop()
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
//    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
//    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
//    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.gx = 0;
    myIMU.gy = 0;
    myIMU.gz = 0;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    //if (myIMU.delt_t > 0.5) //Tiempo de muestreo
   // {
 
// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.+
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;

      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      // Leganes UC3M: 40.332820° N, 3.766701° W
      // 40°19'58.2"N 3°46'00.1"W
      // 0.62° W  ± 0.32°  changing by  0.12° E 
     //myIMU.yaw  -= 8.5;
      myIMU.yaw  += 0.62;
      myIMU.roll *= RAD_TO_DEG;

    // Filtrado de los angulos pitch y roll, 
    // Cuando pitch cambia y alcanza valores cercanos a +-90º(~70), 
    // los valores de roll se disparan, estos deberian mantenerse en 0. 

      if (myIMU.pitch > 70)
      {
        myIMU.roll=0;
      }
      if (myIMU.pitch < -70)
      {
        myIMU.roll=0;
      }

     myIMU.pitch *= DEG_TO_RAD;
     myIMU.roll *= DEG_TO_RAD;
     
     theta=myIMU.pitch;
     phi=myIMU.roll; 

//      if(SerialDebug)
//      {
        // Valores de la libreria del sensor
//        Serial.print("Yaw, Pitch, Roll: ");
//        Serial.print(myIMU.yaw, 2);
//        Serial.print(",");
//        Serial.print(myIMU.pitch, 2);
//        Serial.print(",");
//        Serial.println(myIMU.roll, 2);

        //Valores para orientacion e inclinacion en el cuello.
        // ORIENTACION: Proyeccion en el plano XY del vector Z de la matriz de Tait Bryan. (Yaw =0)
        // INCLINACION: Rotacion del vector Z de la matriz de Tait Bryan, con respecto al eje Z del cuello.
        
        //orientacion con "x"
        
        orient= acos((sin(theta)*cos(phi))/(sqrt((pow(sin(theta),2)*pow(cos(phi),2))+pow(sin(phi),2))));
        orient=orient*180/M_PI;
        if (phi<0)
        {
          orient=360-orient;
        }
        orient=360-orient; // cambio de sentido de giro. Sentido Antiorario. Rotacion segun la referencia del cuello.
        
         //orientacion con "y"
//        orient= acos((-sin(phi))/(sqrt((pow(sin(theta),2)*pow(cos(phi),2))+pow(sin(phi),2))));
//        orient=orient*180/M_PI;
//        Serial.println(orient);

//    Inclinación (proyeccion sobre Z)
        incl= acos((cos(theta)*cos(phi))/(sqrt((pow(sin(theta),2)*pow(cos(phi),2))+pow(sin(phi),2)+(pow(cos(theta),2)*pow(cos(phi),2)))));
        incl=incl*180/M_PI;  
        
        //Serial.print("incl: ");              

//    Offset sensor

      n=sizeof(dataRange)/sizeof(dataRange[0]);
      if (dataRange[99]<=0){ //toma 100 datos para obtener la varianza, media y offset
        if (incl<8){
             i++;
             dataRange[i]=(int16_t)incl;
//             Serial.print(i);
//             Serial.print(",");
//             Serial.println((int16_t)dataRange[i]);
             sum=sum+dataRange[i];                       
        }
         offset=0;
       }       
            mean=(sum/n);
            offset=mean;
            incl=incl-offset;
            if (incl<0)
            {
              incl=0;
            }
            
 
//            Serial.print("offset: ");
//            Serial.println(offset);
//            Serial.print("incl_+off: ");
//            Serial.println((int16_t)incl);

          //Envio de datos a QT
        Serial.print((int16_t)orient);
        Serial.print(",");
        Serial.print((int16_t)incl);
        Serial.print('\n');  
       // Serial.print('\n'); 
        delay(3);


    // With these settings the filter is updating at a ~145 Hz rate using the
    // Madgwick scheme and >200 Hz using the Mahony scheme even though the
    // display refreshes at only 2 Hz. The filter update rate is determined
    // mostly by the mathematical steps in the respective algorithms, the
    // processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
    // an ODR of 10 Hz for the magnetometer produce the above rates, maximum
    // magnetometer ODR of 100 Hz produces filter update rates of 36 - 145 and
    // ~38 Hz for the Madgwick and Mahony schemes, respectively. This is
    // presumably because the magnetometer read takes longer than the gyro or
    // accelerometer reads. This filter update rate should be fast enough to
    // maintain accurate platform orientation for stabilization control of a
    // fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
    // produced by the on-board Digital Motion Processor of Invensense's MPU6050
    // 6 DoF and MPU9150 9DoF sensors. The 3.3 V 8 MHz Pro Mini is doing pretty
    // well!

      myIMU.count = millis();
//      Serial.print(",");
//      Serial.println(myIMU.count);
      myIMU.sumCount = 0;
      myIMU.sum = 0;
   // } // if (myIMU.delt_t > 500)
  } // if (AHRS)
}
