#include <Wire.h>
#include "Kalman.h"
#include "SSD1306Wire.h"
#include <Adafruit_ADS1015.h>

//#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

//-----------------Create Kalman Instances-------------------------//
Kalman kalmanX;
Kalman kalmanY;

//---------------------IMU constants and variables-----------------//
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

//---------------Sharp Sensor constants and Variables-------------//
Adafruit_ADS1115 ads;
int16_t sharp_adc;
float voltage;
double distance;
unsigned long distancetimer;
const int num_samples = 21;
int interval = 200;
double dist_q[num_samples];

//----------------OLED Display variables and constants-----------//
SSD1306Wire  display(0x3c, 0, 2);
unsigned long displaytimer;
//################################################ SETUP #####################################################//
void setup() {

  //----------------------Communication Setup ----------------------//
  Serial.begin(115200);
  Wire.begin(0, 2);
  Wire.setClock(400000L);

  //---------------------------IMU Setup---------------------------//
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(IMUAddress, 0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(IMUAddress, 0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(IMUAddress, 0x75, i2cData, 1));
  if (i2cData[0] != 0x71) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  //----------------------Kalman Filter Setup---------------------//
  
  /* Set kalman and gyro starting angle */
  while (i2cRead(IMUAddress, 0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];
  
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

  //-----------------------ADS 1115 Setup---------------------------//
  ads.begin();
  distancetimer = millis();
  for (int i=0;i<num_samples;i++)
  dist_q[i]=0.0;
  
  //-----------------------OLED Display Setup-----------------------//
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  displaytimer = millis();
}

//########################################## LOOP #########################################//
void loop() {
  //-----------------------Read IMU data--------------------//
  while (i2cRead(IMUAddress, 0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]) + 4000;
  accY = ((i2cData[2] << 8) | i2cData[3]) + 3500;
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  //-------------------------Manipulate Data and Kalman Filter---------------------------//
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

  #ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  #endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  //-------------------------Print Calculated Values----------------------------------//
//    Serial.print(accX); Serial.print("\t");
//    Serial.print(accY); Serial.print("\t");
//    Serial.print(accZ); Serial.print("\t");
//
//    Serial.print(roll); Serial.print("\t");
//    Serial.print(gyroXangle); Serial.print("\t");
//    Serial.print(compAngleX); Serial.print("\t");
//    Serial.print(kalAngleX); Serial.print("\t");
//
//    Serial.print("\t");
//
//    Serial.print(pitch); Serial.print("\t");
//    Serial.print(gyroYangle); Serial.print("\t");
//    Serial.print(compAngleY); Serial.print("\t");
//    Serial.print(kalAngleY); Serial.print("\t");
//
//    Serial.print("\t");
//
//    double temperature = (double)tempRaw / 340.0 + 36.53;
//    Serial.print(temperature); Serial.print("\t");
//
//    Serial.print("\t");
    
    
    sharp_adc = ads.readADC_SingleEnded(0);
    voltage = sharp_adc*0.1875/1000;
    distance = 13*pow(voltage,-1);
    if (((millis() - distancetimer) > interval) && distance > 4 && distance < 20){
      for (int  i= (num_samples-1); i>0; i--){
        dist_q[i] = dist_q[i-1];
      }
      dist_q[0] = distance;
      Serial.print("[ ");
      for (int i = (num_samples-1); i>= 0; i--){
        Serial.print(dist_q[i]);
        if (i!=0)Serial.print(", "); 
      }
      Serial.print(" ]");Serial.println("");
      
      double max = dist_q[0];
      int max_pos = 0;
      for (int i = 0; i < num_samples; i++)
      {
       if (dist_q[i] > max){
          max = dist_q[i];
          max_pos = i;
       }
      }
      
      double diff_forward = 0.0;
      double diff_backward = 0.0;
      Serial.print(diff_forward);Serial.print("\t");
      Serial.print(diff_backward);Serial.print("\t");
      //back swipe
      for (int i=0; i<max_pos ; i++){
        diff_backward += (dist_q[i+1] - dist_q[i]);
      }

      //forward swipe
      for (int i=max_pos; i<num_samples; i++){
        diff_forward += (dist_q[i-1] - dist_q[i]);
      }

      if (diff_forward > 4 && diff_backward > 4 && diff_forward < 7 && diff_backward <7)
        Serial.print("SHORT SWIPE SHORT SWIPE SHORT SWIPE SHORT SWIPE");
      else if(diff_forward >= 10 && diff_backward >= 10 && diff_forward < 20 && diff_backward <20)
        Serial.print("LONG SWIPE LONG SWIPE LONG SWIPE LONG SWIPE");
        
      distancetimer = millis();
      Serial.print(distance);Serial.print("\t");
      Serial.print(diff_forward);Serial.print("\t");
      Serial.print(diff_backward);Serial.print("\t");
      Serial.print(max);Serial.print("\t");
      Serial.print(max_pos);Serial.println("");
    }
//    else{
//      Serial.println(0);
//    }
    

    
    //printondisplay(roll, kalAngleX, pitch, kalAngleY, distance);
    
  //Serial.print("\r\n");
  delay(2);
}


//################################################## HELPER FUNCTIONS ##############################################//

//---------------------------I2C Communication Helper Functions-------------------------------//
uint8_t i2cWrite(uint8_t DeviceAddress, uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(DeviceAddress, registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t DeviceAddress, uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(DeviceAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t DeviceAddress, uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(DeviceAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(DeviceAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}

//------------------------------------OLED display helper functions-----------------------------------//
void printondisplay(double roll, double kalAngleX, double pitch, double kalAngleY, double distance){
    if (millis() - displaytimer > 500){
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);

    String temp1_String = String(roll, 2);
    String temp2_String = String(pitch, 2);
    String rollpitchString = temp1_String + " " + temp2_String + "\n";

    temp1_String = String(kalAngleX, 2);
    temp2_String = String(kalAngleY, 2);
    String kalmanString = temp1_String + " " + temp2_String + "\n";

    temp1_String = String(distance, 2);
    String distanceString = temp1_String + " cm" + "\n";
    display.drawString(0, 10, rollpitchString);
    display.drawString(0, 20, kalmanString);
    display.drawString(0, 30, distanceString);
    display.display();
    
    displaytimer = millis();
    }
}

